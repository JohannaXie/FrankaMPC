// Standard
#include <pthread.h>
#include <chrono>  // timer
#include <cmath>
#include <iostream>

// Franka
#include <franka/exception.h>
#include <franka/robot.h>

#include "examples_common.h"

// CVXGEN
#include "solver.h"

#define NUM_JOINTS 7
#define NUM_DEPLOY 2  // Number of MPC recalculations
#define N 25           // Prediction horizon
#define T 2            // Allowed time

using namespace std;
using namespace std::chrono;

// CVXGEN variable declaration (memory allocation)
Vars vars;
Params params;
Workspace work;
Settings settings;

// Multi-thread variable declaration
pthread_mutex_t mtx;
bool stopFlag;

int init_robot(franka::Robot& robot);
void init_mpc();
void* MPC(void* arg);
void* control(void* arg);
void load_joint_data(int joint);
double interpolate_command(int joint, double time);

// Structs
struct states {
  double q, v, a;  // Every state has pos, vel, acc
};

struct joints {
  struct states x[NUM_JOINTS];  // All joints have a state each
};

struct commands {
  struct joints j[N+1];  // Every command has N "joints"
  int n;
} c;

volatile double runTime = 0.0;
volatile double h = (double)T / (double)N;
volatile double maxTime = 2.0;
volatile double timeLeft = maxTime - runTime;

int n = 0;

int main(int argc, char** argv) {
  if (argc != 2) {
    std::cerr << "Usage: " << argv[0] << " <robot-hostname>" << std::endl;
    return -1;
  }
  stopFlag = false;

  init_mpc();

  pthread_t ctrl_t, mpc_t;
  pthread_mutex_init(&mtx, NULL);
  pthread_create(&ctrl_t, NULL, control, NULL);
  pthread_create(&mpc_t, NULL, MPC, NULL);

  while (!stopFlag) {
    usleep(1000);
  }

  pthread_exit(&ctrl_t);
  pthread_exit(&mpc_t);
  pthread_mutex_destroy(&mtx);
  return 0;
}

int init_robot(franka::Robot& robot) {
  try {
    setDefaultBehavior(robot);

    std::array<double, 7> q_goal = {{0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_2}};
    MotionGenerator motion_generator(0.5, q_goal);
    std::cout << "Press ENTER to move robot to starting position" << std::endl;
    std::cin.ignore();
    robot.control(motion_generator);
    std::cout << "Finished moving to initial joint configuration." << std::endl;

    robot.setCollisionBehavior(
        {{200.0, 200.0, 180.0, 180.0, 160.0, 140.0, 120.0}},
        {{200.0, 200.0, 180.0, 180.0, 160.0, 140.0, 120.0}},
        {{200.0, 200.0, 180.0, 180.0, 160.0, 140.0, 120.0}},
        {{200.0, 200.0, 180.0, 180.0, 160.0, 140.0, 120.0}},
        {{200.0, 200.0, 200.0, 250.0, 250.0, 250.0}}, {{200.0, 200.0, 200.0, 250.0, 250.0, 250.0}},
        {{200.0, 200.0, 200.0, 250.0, 250.0, 250.0}}, {{200.0, 200.0, 200.0, 250.0, 250.0, 250.0}});
    return 0;
  } catch (const franka::Exception& e) {
    std::cout << e.what() << std::endl;
    return -1;
  }
}

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
  params.rf[1] = 0.0;
  params.rf[2] = 0.0;
}

void* MPC(void* arg) {
  std::array<double, 7> q_start = {{0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_2}};
  std::array<double, 7> q_target = {{M_PI_4, -M_PI_4, 0, -2 * M_PI_4, 0, M_PI_2, M_PI_4}};
  int deployCount = 0;
  double h = (double)T / (double)N;
  while (!stopFlag && deployCount<NUM_DEPLOY) {
    double deployTime = deployCount * (double)T / (double)NUM_DEPLOY - 0.010;
    
    if (runTime > deployTime) {
      deployCount++;
      std::cout << "Deploy count: " << deployCount << "\t runTime: " << runTime << std::endl;
      h = timeLeft / N;
      
      calculate_transfer_matrix(h);
      int j;
      for (j = 0; j < NUM_JOINTS; j++) {
        load_joint_data(j);
        if (runTime>0.0)
        {
          q_start[j] = interpolate_command(j, runTime);
        }
        params.x_0[0] = q_start[j];
        params.x_0[1] = 0.0;
        params.x_0[2] = 0.0;
        params.rf[0] = q_target[j];
        if (j == 6) {
          std::cout << q_start[6] << std::endl;
        }

        solve();

        int i;
        for (i = 1; i <= N; i++) {
          c.j[i - 1].x[j].q = vars.x[i][0];
          std::cout << "Joint: " << j << " Step: " << i << " Value: " << c.j[i - 1].x[j].q << std::endl;
        }
      }
    }
  }

  pthread_exit(NULL);
}

void* control(void* arg) {
  franka::Robot robot("172.16.0.2");
  init_robot(robot);
  try {
    runTime = 0.0;
    timeLeft = maxTime - runTime;
    std::array<double, 7> q_start = {{0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_2}};
    std::array<double, 7> q_target = {{M_PI_4, -M_PI_4, 0, -2 * M_PI_4, 0, M_PI_2, M_PI_4}};

    struct joints ctrl_old;

    for (size_t i = 0; i < NUM_JOINTS; i++) {
      ctrl_old.x[i].q = q_start[i];
      c.j[25].x[i].q = q_target[i];
    }

    robot.control(
        [&q_start, &q_target, &ctrl_old](const franka::RobotState& robot_state,
                                         franka::Duration period) -> franka::JointPositions {
          auto start = high_resolution_clock::now();
          runTime += period.toSec();
          timeLeft = maxTime - runTime;

          if (runTime == 0.0) {
            q_start = robot_state.q_d;
          }
          n = N-(int)(timeLeft / h);
          std::cout << "n: " << n << std::endl;
          if (n < 25) {
            for (size_t i = 6; i < NUM_JOINTS; i++) {
              q_start[i] = c.j[n].x[i].q + (c.j[n + 1].x[i].q - c.j[n].x[i].q) * (runTime / h - n);
              //q_start[i] -= delta;
            }
          }

          franka::JointPositions output = {
              {q_start[0], q_start[1], q_start[2], q_start[3], q_start[4], q_start[5], q_start[6]}};

          if (runTime >= maxTime) {
            std::cout << "Finished motion, shutting down example" << std::endl;
            stopFlag = true;
            return franka::MotionFinished(output);
          }
          auto stop = high_resolution_clock::now();
          auto duration = duration_cast<microseconds>(stop - start);
          return output;
        });

  } catch (const franka::Exception& e) {
    std::cerr << e.what() << '\n';
  }
  pthread_exit(NULL);
}

void load_joint_data(int joint) {
  switch (joint) {
    case 0:
      params.x_max[0] = 2.8973;   // q_max
      params.x_min[0] = -2.8973;  // q_min
      params.x_max[1] = 2.1750;   // qdot_max
      params.x_min[1] = -2.1750;
      params.x_max[2] = 15.0;  // qdotdot_max
      params.x_min[2] = -15.0;
      params.u_max[0] = 7500.0;
      break;
    case 1:
      params.x_max[0] = 1.7628;   // q_max
      params.x_min[0] = -1.7628;  // q_min
      params.x_max[1] = 2.1750;   // qdot_max
      params.x_min[1] = -2.1750;
      params.x_max[2] = 7.5;  // qdotdot_max
      params.x_min[2] = -7.5;
      params.u_max[0] = 3750.0;
      break;
    case 2:
      params.x_max[0] = 2.8973;   // q_max
      params.x_min[0] = -2.8973;  // q_min
      params.x_max[1] = 2.1750;   // qdot_max
      params.x_min[1] = -2.1750;
      params.x_max[2] = 10.0;  // qdotdot_max
      params.x_min[2] = -10.0;
      params.u_max[0] = 5000.0;
      break;
    case 3:
      params.x_max[0] = -0.0698;  // q_max
      params.x_min[0] = -3.0718;  // q_min
      params.x_max[1] = 2.1750;   // qdot_max
      params.x_min[1] = -2.1750;
      params.x_max[2] = 12.5;  // qdotdot_max
      params.x_min[2] = -12.5;
      params.u_max[0] = 6250.0;
      break;
    case 4:
      params.x_max[0] = 2.8973;   // q_max
      params.x_min[0] = -2.8973;  // q_min
      params.x_max[1] = 2.6100;   // qdot_max
      params.x_min[1] = -2.6100;
      params.x_max[2] = 15.0;  // qdotdot_max
      params.x_min[2] = -15.0;
      params.u_max[0] = 7500.0;
      break;
    case 5:
      params.x_max[0] = 3.7525;   // q_max
      params.x_min[0] = -0.0175;  // q_min
      params.x_max[1] = 2.6100;   // qdot_max
      params.x_min[1] = -2.6100;
      params.x_max[2] = 20.0;  // qdotdot_max
      params.x_min[2] = -20.0;
      params.u_max[0] = 10000.0;
      break;
    case 6:
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

double interpolate_command(int joint, double time) {
  //std::cout << "n: " << n << "joint: " << joint << std::endl;
  std::cout << c.j[n].x[joint].q << std::endl;
  return c.j[n].x[joint].q + (c.j[n + 1].x[joint].q - c.j[n].x[joint].q) * (time / h - n);
}
