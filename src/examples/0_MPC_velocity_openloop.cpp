// libfranka includes
#include <franka/exception.h>
#include <franka/robot.h>

// Multi-thread includes
#include <pthread.h>

// Standard includes
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <chrono>
#include <cmath>
#include <iostream>

// libfranka includes
#include "examples_common.h"

//
#include "solver.h"

/**
 * Author: FRTN40 Group F (Yixi Cai, Simon Ågren, Yuhan Xie, Meghan Young)
 *
 * 2020-01-08
 * @warning Before executing this example, make sure there is enough space surounding the robot. Move away from the robot after having disturbed it.
 */

#define T 5              // Given time, [s]
#define N 25             // Prediction horizon
#define StableTime 0.01  // Time for robot to reach stable state [s]

// Gain used for tuning of velocity commands
#define Gain0 0.935
#define Gain1 1.0
#define Gain2 1.0
#define Gain3 1.0
#define Gain4 1.06
#define Gain5 1.1
#define Gain6 1.12

#define NUM_JOINTS 7

using namespace std;
using namespace std::chrono;

struct state {                // State (position, velocity, acceleration)
  double q, v, a;
};

struct CTRL_MSG {             // Control message
  struct state x_old[NUM_JOINTS], x[NUM_JOINTS];  // Current and previous states
  double stamp, h;            // Time stamp, Sample time
} ctrl_msg;

struct MPC_MSG {
  double h, stamp;            // Sample time, Time stamp
  struct state x[NUM_JOINTS]; // States
  float rf[NUM_JOINTS];       // Reference (target)
} mpc_msg;

// A "standard" initial pose that is safe to move to
std::array<double, NUM_JOINTS> initial_pose = {{0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4}};

// CVXGEN variable declaration (memory allocation)
Vars vars;
Params params;
Workspace work;
Settings settings;

// Multi-thread variable declaration
pthread_mutex_t mtx;
bool StopFlag = false;

// Data recording and accuracy calculation
FILE* mpc_fp;
FILE* ctrl_fp;
float reference[NUM_JOINTS]; 
double q_final[NUM_JOINTS];

// Robot & timer initiation
franka::Robot robot("172.16.0.2");
double timer = 0.0, maxTime = 0.0;

/* 
Robot initialization function
*/
void init_robot() {
  try {
    setDefaultBehavior(robot);
    MotionGenerator motion_generator(0.5, initial_pose);
    robot.control(motion_generator);
    std::cout << "Finished moving to initial joint configuration." << std::endl;

    // Collision behavior - high values (x10 default) enable perturbation without throw of Exception (interruption of script)
    robot.setCollisionBehavior(
        {{200.0, 200.0, 180.0, 180.0, 160.0, 140.0, 120.0}},
        {{200.0, 200.0, 180.0, 180.0, 160.0, 140.0, 120.0}},
        {{200.0, 200.0, 180.0, 180.0, 160.0, 140.0, 120.0}},
        {{200.0, 200.0, 180.0, 180.0, 160.0, 140.0, 120.0}},
        {{200.0, 200.0, 200.0, 250.0, 250.0, 250.0}}, {{200.0, 200.0, 200.0, 250.0, 250.0, 250.0}},
        {{200.0, 200.0, 200.0, 250.0, 250.0, 250.0}}, {{200.0, 200.0, 200.0, 250.0, 250.0, 250.0}});
  } catch (const franka::Exception& e) {
    std::cout << e.what() << std::endl;
  }
}

/* 
Start and reference/target pose reading function 
*/
void init_poses() {
  // Read target position
  std::cout << "MOVE the robot to TARGET position and press Enter to continue..." << std::endl;
  std::cin.ignore();
  franka::RobotState state = robot.readOnce();
  for (int i = 0; i < NUM_JOINTS; i++) {
    reference[i] = state.q[i];    // Save to reference for accuracy calculation
    mpc_msg.rf[i] = state.q[i];   // MPC reference
  }

  // Read starting position
  std::cout << "MOVE the robot to STARTING position and press Enter to continue..." << std::endl;
  std::cin.ignore();
  state = robot.readOnce();
  for (int i = 0; i < NUM_JOINTS; i++) {
    initial_pose[i] = state.q[i]; // Save starting pose
    mpc_msg.x[i].q = initial_pose[i]; // Provide MPC with this pose
    mpc_msg.x[i].v = 0.0;         // Initial speed is 0
    mpc_msg.x[i].a = 0.0;         // Initial acceleration is 0
  }

  // For testing to see if START and TARGET pose reading worked
  std::cout << "(Optional) MOVE the robot to ANY position and press Enter to continue..."
            << std::endl;
  std::cin.ignore();
}

/* 
CVXGEN/MPC initialization 
*/
void init_mpc() {
  set_defaults();        // Set basic algorithm parameters.
  setup_indexing();      // Allows parameters to be accessed as params.y[1][j] = ... instead of
                         // params.y_1[j] = ...
  settings.verbose = 0;  // Disable output of solver progress

  // Load weight matrices Q and R
  params.Q[0] = 0.0;
  params.Q[3] = 0.0;
  params.Q[6] = 0.0;
  params.Q[1] = 0.0;
  params.Q[4] = 1.0;
  params.Q[7] = 0.0;
  params.Q[2] = 0.0;
  params.Q[5] = 0.0;
  params.Q[8] = 1.0;
  params.R[0] = 0.001;
  // Initialize sample time and time stamp in MPC
  mpc_msg.h = (T-timer)/N;
  mpc_msg.stamp = 0.0;
}

/* 
Control & robot communication thread
*/
void* control_thread(void* arg) {
  try {
    std::array<double, NUM_JOINTS> tau_ext; // External torques
    double h = (T - timer) / N;             // Sample time initialization in control

    // Perturbation variables
    bool pertFlag = false;
    bool firstPert = true;
    int pertCount = 0;

    // Initialization of timers (started here only to be able to use "auto", restarted later)
    auto start = high_resolution_clock::now(); 
    auto stop = high_resolution_clock::now();  
    double pertTime = 0;

    // State and control variables
    struct state x[NUM_JOINTS], x_old[NUM_JOINTS], x_control[NUM_JOINTS], x_control_last[NUM_JOINTS];

    // BEGIN Control loop, executed at 1 kHz by definition
    robot.control([&x, &x_old, &x_control, &x_control_last, &h, &tau_ext, &pertFlag, &pertCount,
                   &start, &stop, &pertTime,
                   &firstPert](const franka::RobotState& robot_state,
                               franka::Duration period) -> franka::JointVelocities {
      double stamp, h_interpolation;
      struct state x_pred[NUM_JOINTS];
      franka::JointVelocities output = {{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
      timer += period.toSec();  // Update timer
      h = (T - timer) / N;      // Update sample time accordingly
      std::array<double, NUM_JOINTS> tau_ext;
      tau_ext = robot_state.tau_ext_hat_filtered; // Read external torques applied on joints

      // Action is assumed to be performed at the gripper, so only joint 5 & 6 are checked
      if (std::abs(tau_ext[6]) > 0.7 || std::abs(tau_ext[5]) > 0.5) {
        // For testing purposes
        /*
        if (std::abs(tau_ext[6]) > 0.7) {
          std::cout << "Joint 6 was disturbed" << std::endl;
        } else {
          std::cout << "Joint 5 was disturbed" << std::endl;
        }
        */
        pertFlag = true;
        pertCount = 0;

        // Start timer at first detected perturbation, because they tend to not be continuous
        if (firstPert) {
          start = high_resolution_clock::now();
          firstPert = false;
        }
      }

      if (pertFlag) {  // Mode 1: Perturbation mode
        // Manual torque control (torque -> velocity)
        tau_ext = robot_state.tau_ext_hat_filtered;
        for (size_t i = 0; i < NUM_JOINTS; i++) {
          output.dq[i] = -0.01 * tau_ext[i];
        }
        // Read robot states and update MPC 
        send_mpc_params(robot_state.q, robot_state.dq, robot_state.ddq_d, h, timer);
        
        // If not perturbed any more
        if (std::abs(tau_ext[6]) < 0.7 && std::abs(tau_ext[5]) < 0.5) {
          pertCount++;

          // For testing purposes
          // std::cout << pertCount << std::endl;

          // Pause after last perturbation, allow user to move away
          if (pertCount > 500) {
            stop = high_resolution_clock::now();
            pertTime += duration_cast<duration<double>>(stop - start).count();

            // Deduct perturbation time from timer. Not neccesary.
            // timer -=  pertTime;

            // Reset flag and count
            pertFlag = false;
            pertCount = 0;
          }
        }
      } else {  // Mode 2: "Normal" MPC mode
        load_ctrl_params(x, x_old, &h_interpolation, &stamp);
        fprintf(ctrl_fp, " %f %f", timer, h_interpolation);
        interpolate(x, x_old, x_control, h_interpolation, stamp);
        int i;
        for (i = 0; i < 7; i++) {
          if (timer < T)
            x_control_last[i] = x_control[i];
          else
            x_control_last[i].v = 0.0;
          fprintf(ctrl_fp, " %f", robot_state.q_d[i]);
          fprintf(ctrl_fp, " %f", robot_state.dq_d[i]);
          fprintf(ctrl_fp, " %f", robot_state.ddq_d[i]);
        }
        fprintf(ctrl_fp, "\n");
        output = {{Gain0 * x_control_last[0].v, Gain1 * x_control_last[1].v,
                   Gain2 * x_control_last[2].v, Gain3 * x_control_last[3].v,
                   Gain4 * x_control_last[4].v, Gain5 * x_control_last[5].v,
                   Gain6 * x_control_last[6].v}};

        interpolate(x, x_old, x_pred, h_interpolation, stamp - 0.001);
        send_mpc_params_openloop(x_pred, h, timer);
      }

      // When time is up
      if (timer >= T + StableTime && !pertFlag) {
        std::cout << "Perturbation time: " << pertTime << std::endl;
        std::cout << "Finished motion" << std::endl;
        StopFlag = true;
        int i;
        for (i = 0; i < 7; i++) {
          q_final[i] = robot_state.q_d[i];
        }
        return franka::MotionFinished(output);
      }
      return output;
    }); // END Control loop
  } catch (const franka::Exception& e) {
    std::cout << e.what() << std::endl;
  }
  pthread_exit(NULL); // Terminate control-thread
}

/*
MPC thread
*/
void* MPC_thread(void* arg) {
  struct state x[NUM_JOINTS];
  struct state x_old[NUM_JOINTS];
  double h = 0.0, Time = 0.0;

  while(!StopFlag) { // While the control loop is running, i.e. not out of time
    auto start = high_resolution_clock::now(); // For calculating MPC calculation time
    int i;
    for (i = 0; i < NUM_JOINTS; i++) {
      load_joint_data(i);   // Joint dependent params such as position limits

      /* Possible improvement: Move transfer matrix (inside this function)
      outside this for loop. Same transfer function for all
      joints. It only depends on the sample time h */
      load_mpc_params(i, x_old, &h, &Time);

      if (Time >= T)
        break;
      solve();    // Run optimizing code
      x[i].q = vars.x_1[0];
      x[i].v = vars.x_1[1];
      x[i].a = vars.x_1[2];

      // printf functions for Matlab plots
      if (i == 0)
        fprintf(mpc_fp, "%f %f", Time, h);

      // Trajectory plots
      /*if (i == 3) {
        fprintf(mpc_fp, " %f %f %f %f %f %f %f %f %f %f", vars.x_1[0], vars.x_2[0], vars.x_3[0],
                vars.x_4[0], vars.x_5[0], vars.x_6[0], vars.x_7[0], vars.x_8[0], vars.x_9[0],
                vars.x_10[0]);
        fprintf(mpc_fp, " %f %f %f %f %f %f %f %f %f %f", vars.x_11[0], vars.x_12[0], vars.x_13[0],
                vars.x_14[0], vars.x_15[0], vars.x_16[0], vars.x_17[0], vars.x_18[0], vars.x_19[0],
                vars.x_20[0]);
        fprintf(mpc_fp, " %f %f %f %f %f", vars.x_21[0], vars.x_22[0], vars.x_23[0], vars.x_24[0],
                vars.x_25[0]);
      }
      */
      // Pos, vel, acc plots 
      fprintf(mpc_fp, " %f", x[i].q);
      fprintf(mpc_fp, " %f", x[i].v);
      fprintf(mpc_fp, " %f", x[i].a);
    }
    if (Time <= T)
      fprintf(mpc_fp, "\n");
    send_ctrl_params(x, x_old, Time, h);
    auto stop = high_resolution_clock::now();

    // Calculate MPC duration
    auto duration = duration_cast<microseconds>(stop - start);
    usleep(max(0, int(10000 - duration.count())));

    maxTime = max(maxTime, duration.count() / 1000.0); // Save longest duration
  } 
  pthread_exit(NULL); // Terminate MPC-thread
}

int main(int argc, char** argv) {
  double err = 0.0;   // Angle error

  // Data recording for Matlab plots
  mpc_fp = fopen("MPC_DataRecord.txt", "w+");
  ctrl_fp = fopen("Control_DataRecord.txt", "w+");
  printf("%d\n", ctrl_fp);

  // Initialiation functions
  init_poses();   
  init_robot();  
  init_mpc();

  // Initialize threads
  pthread_t ctrl_t, mpc_t;
  pthread_mutex_init(&mtx, NULL);
  pthread_create(&ctrl_t, NULL, control_thread, NULL);
  pthread_create(&mpc_t, NULL, MPC_thread, NULL);

  // Do nothing while script is running
  while (!StopFlag) {
    usleep(1000);
  }

  // Print accuracy results
  printf("------------DONE-------------\n");
  for (int i = 0; i < 7; i++) {
    if (fabs(q_final[i] - reference[i]) < 1e-6)
      err = 0;
    else
      err = (q_final[i] - reference[i]);
    printf("Joint %d:\n  Reference:%0.5f deg,\tResult:%0.5f deg,\tError:%0.5f deg\n", i,
           reference[i] / M_PI * 180, q_final[i] / M_PI * 180, err / M_PI * 180);
  }
  printf("Max MPC calculation time was %0.4fms \n", maxTime);

  // Terminate threads, locks and close files
  pthread_exit(&ctrl_t);
  pthread_exit(&mpc_t);
  pthread_mutex_destroy(&mtx);
  fclose(mpc_fp);
  fclose(ctrl_fp);

  return 0;
}

// Load joint constraints 
void load_joint_data(int joint) {
  switch (joint + 1) {
    case 1:
      params.x_max[0] = 2.8973;   // q_max
      params.x_min[0] = -2.8973;  // q_min
      params.x_max[1] = 2.1750;   // qdot_max
      params.x_min[1] = -2.1750;
      params.x_max[2] = 15.0;  // qdotdot_max
      params.x_min[2] = -15.0;
      params.u_max[0] = 7500.0;
      break;
    case 2:
      params.x_max[0] = 1.7628;   // q_max
      params.x_min[0] = -1.7628;  // q_min
      params.x_max[1] = 2.1750;   // qdot_max
      params.x_min[1] = -2.1750;
      params.x_max[2] = 7.5;  // qdotdot_max
      params.x_min[2] = -7.5;
      params.u_max[0] = 3750.0;
      break;
    case 3:
      params.x_max[0] = 2.8973;   // q_max
      params.x_min[0] = -2.8973;  // q_min
      params.x_max[1] = 2.1750;   // qdot_max
      params.x_min[1] = -2.1750;
      params.x_max[2] = 10.0;  // qdotdot_max
      params.x_min[2] = -10.0;
      params.u_max[0] = 5000.0;
      break;
    case 4:
      params.x_max[0] = -0.0698;  // q_max
      params.x_min[0] = -3.0718;  // q_min
      params.x_max[1] = 2.1750;   // qdot_max
      params.x_min[1] = -2.1750;
      params.x_max[2] = 12.5;  // qdotdot_max
      params.x_min[2] = -12.5;
      params.u_max[0] = 6250.0;
      break;
    case 5:
      params.x_max[0] = 2.8973;   // q_max
      params.x_min[0] = -2.8973;  // q_min
      params.x_max[1] = 2.6100;   // qdot_max
      params.x_min[1] = -2.6100;
      params.x_max[2] = 15.0;  // qdotdot_max
      params.x_min[2] = -15.0;
      params.u_max[0] = 7500.0;
      break;
    case 6:
      params.x_max[0] = 3.7525;   // q_max
      params.x_min[0] = -0.0175;  // q_min
      params.x_max[1] = 2.6100;   // qdot_max
      params.x_min[1] = -2.6100;
      params.x_max[2] = 20.0;  // qdotdot_max
      params.x_min[2] = -20.0;
      params.u_max[0] = 10000.0;
      break;
    case 7:
      params.x_max[0] = 2.8973;   // q_max
      params.x_min[0] = -2.8973;  // q_min
      params.x_max[1] = 2.6100;   // qdot_max
      params.x_min[1] = -2.6100;
      params.x_max[2] = 20.0;  // qdotdot_max
      params.x_min[2] = -20.0;
      params.u_max[0] = 10000.0;
      break;
    default:
      break;
  }
}

// Load MPC parameters
void load_mpc_params(int joint, struct state x_old[], double* h, double* stamp) {
  pthread_mutex_lock(&mtx);
  calculate_transfer_matrix(mpc_msg.h);
  *h = mpc_msg.h;
  *stamp = mpc_msg.stamp;
  params.x_0[0] = mpc_msg.x[joint].q;
  params.x_0[1] = mpc_msg.x[joint].v;
  params.x_0[2] = mpc_msg.x[joint].a;
  // printf("%f %f %f\n", params.x_0[0], params.x_0[1], params.x_0[2]);
  x_old[joint] = mpc_msg.x[joint];
  params.rf[0] = mpc_msg.rf[joint];
  pthread_mutex_unlock(&mtx);
  params.rf[1] = 0.0;     // Speed at TARGET should be 0
  params.rf[2] = 0.0;     // Acceleration at TARGET should be 0
}

void load_ctrl_params(struct state x[],
                      struct state x_old[],
                      double* h_interpolation,
                      double* stamp) {
  int i;
  pthread_mutex_lock(&mtx);
  for (i = 0; i < 7; i++) {
    x[i] = ctrl_msg.x[i];
    x_old[i] = ctrl_msg.x_old[i];
  }
  *h_interpolation = ctrl_msg.h;
  *stamp = ctrl_msg.stamp;
  pthread_mutex_unlock(&mtx);
}

void send_ctrl_params(struct state x[], struct state x_old[], double stamp, double h) {
  pthread_mutex_lock(&mtx);
  int i;
  ctrl_msg.stamp = stamp;
  ctrl_msg.h = h;
  for (i = 0; i < 7; i++) {
    ctrl_msg.x[i] = x[i];
    ctrl_msg.x_old[i] = x_old[i];
  }
  pthread_mutex_unlock(&mtx);
}

void send_mpc_params(std::array<double, 7> q,
                     std::array<double, 7> v,
                     std::array<double, 7> a,
                     double h,
                     double stamp) {
  int i;
  pthread_mutex_lock(&mtx);
  mpc_msg.h = h;
  mpc_msg.stamp = stamp;
  for (i = 0; i < 7; i++) {
    mpc_msg.x[i].q = q[i];
    mpc_msg.x[i].v = v[i];
    mpc_msg.x[i].a = a[i];
  }
  pthread_mutex_unlock(&mtx);
}

void send_mpc_params_openloop(struct state x[], double h, double stamp) {
  int i;
  pthread_mutex_lock(&mtx);
  mpc_msg.h = h;
  mpc_msg.stamp = stamp;
  for (i = 0; i < 7; i++) {
    mpc_msg.x[i] = x[i];
  }
  pthread_mutex_unlock(&mtx);
}

/* 
Interpolation function 
*/
void interpolate(struct state x[],
                 struct state x_old[],
                 struct state x_control[],
                 double h,
                 double stamp) {
  int i;
  if (h > 0.0) {
    for (i = 0; i < 7; i++) {
      x_control[i].q = x_old[i].q + (x[i].q - x_old[i].q) / h * (timer - stamp);
      x_control[i].v = x_old[i].v + (x[i].v - x_old[i].v) / h * (timer - stamp);
      x_control[i].a = x_old[i].a + (x[i].a - x_old[i].a) / h * (timer - stamp);
    }
  } else {
    for (i = 0; i < 7; i++) {
      x_control[i].q = initial_pose[i];
      x_control[i].v = 0.0;
      x_control[i].a = 0.0;
    }
  }
}

/* 
Calculate transfer matrix for MPC
*/
void calculate_transfer_matrix(double h) {
  params.A[0] = 1.0;
  params.A[1] = 0.0;
  params.A[2] = 0.0;
  params.A[3] = h;
  params.A[4] = 1.0;
  params.A[5] = 0.0;
  params.A[6] = h * h * 0.5;
  params.A[7] = h;
  params.A[8] = 1.0;
  params.B[0] = 7 * h * h * h / 12.0;
  params.B[1] = h * h;
  params.B[2] = h;
  params.Gamma_1h[0] = h * h * h / 24;
  params.Gamma_1h[1] = h * h / 6;
  params.Gamma_1h[2] = h / 2;
}
