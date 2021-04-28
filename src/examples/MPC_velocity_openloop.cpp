
// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <franka/exception.h>
#include <franka/robot.h>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <chrono>
#include <cmath>
#include <iostream>
#include "examples_common.h"
#include "solver.h"

/**
 * Author:
 *
 * Date
 * @warning Before executing this example, make sure there is enough space in front of the robot.
 */

#define T 2              // Given time, unit: sec
#define N 25             // Prediction horizon
#define StableTime 0.01  // Time for robot to reach stable state, unit:sec

using namespace std;
using namespace std::chrono;

struct state {
  double q, v, a;
};

struct CTRL_MSG {
  struct state x_old[7], x[7];
  double stamp, h;
} ctrl_msg;

struct MPC_MSG {
  double h, stamp;
  struct state x[7];
  float rf[7];
} mpc_msg;

// const double reference[7] = {0, -M_PI_4, M_PI_4/4, -3 * M_PI_4 - M_PI_4/4, -M_PI_4, M_PI_2,
// M_PI_4};
float reference[7];
const double initial_pose[7] = {0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4};
// Variables definition for CVXGEN

Vars vars;
Params params;
Workspace work;
Settings settings;

// Definition for multi-thread

pthread_mutex_t mtx;
bool StopFlag = false;

// File record

FILE* mpc_fp;
FILE* ctrl_fp;

franka::Robot robot("172.16.0.2");

double timer = 0.0, maxTime = 0.0;

double q_final[7];  // record the result of MPC

void* control_thread(void* arg) {
  try {
    setDefaultBehavior(robot);
    // First move the robot to a suitable joint configuration
    std::array<double, 7> q_goal = {{0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4}};
    MotionGenerator motion_generator(0.5, q_goal);
    robot.control(motion_generator);
    std::cout << "Finished moving to initial joint configuration." << std::endl;

    // Set additional parameters always before the control loop, NEVER in the control loop!
    // Set collision behavior.
    robot.setCollisionBehavior(
        {{200.0, 200.0, 180.0, 180.0, 160.0, 140.0, 120.0}},
        {{200.0, 200.0, 180.0, 180.0, 160.0, 140.0, 120.0}},
        {{200.0, 200.0, 180.0, 180.0, 160.0, 140.0, 120.0}},
        {{200.0, 200.0, 180.0, 180.0, 160.0, 140.0, 120.0}},
        {{200.0, 200.0, 200.0, 250.0, 250.0, 250.0}}, {{200.0, 200.0, 200.0, 250.0, 250.0, 250.0}},
        {{200.0, 200.0, 200.0, 250.0, 250.0, 250.0}}, {{200.0, 200.0, 200.0, 250.0, 250.0, 250.0}});

    // robot.control
    double h = 0.0;
    std::array<double, 7> ext_torque;
    struct state x[7], x_old[7], x_control[7], x_control_last[7];
    robot.control([&x, &x_old, &x_control, &x_control_last, &timer, &ext_torque, &h](const franka::RobotState& robot_state, franka::Duration period) -> franka::JointVelocities {
      int i;
      double stamp, h_interpolation;
      struct state x_pred[7];
      timer += period.toSec();
      ext_torque = robot_state.tau_ext_hat_filtered;
      for (i = 0; i < 7; i++) {
        if (abs(ext_torque[i]) > 2)
          printf("external torque %d: %f\n", i, ext_torque[i]);
      }
      h = (T - timer) / N;
      load_ctrl_params(x, x_old, &h_interpolation, &stamp);
      fprintf(ctrl_fp, " %f %f", timer, h_interpolation);
      interpolate(x, x_old, x_control, h_interpolation, stamp);

      for (i = 0; i < 7; i++) {
        if (timer < T)
          x_control_last[i] = x_control[i];
        else
          x_control_last[i].v = 0.0;
        fprintf(ctrl_fp, " %f", robot_state.q_d[i]);
      }
      fprintf(ctrl_fp, "\n");
      franka::JointVelocities output = {
          {x_control_last[0].v, x_control_last[1].v, x_control_last[2].v, x_control_last[3].v,
           x_control_last[4].v, x_control_last[5].v, x_control_last[6].v}};
      interpolate(x, x_old, x_pred, h_interpolation, stamp - 0.001);
      send_mpc_params_openloop(x_pred, h, timer);
      if (timer >= T + StableTime) {
        std::cout << std::endl << "Finished motion" << std::endl;
        StopFlag = true;
        for (i = 0; i < 7; i++) {
          q_final[i] = robot_state.q_d[i];
        }
        return franka::MotionFinished(output);
      }
      return output;
    });
  } catch (const franka::Exception& e) {
    std::cout << e.what() << std::endl;
  }
  pthread_exit(NULL);
}

void* MPC_thread(void* arg) {
  struct state x[7];
  struct state x_old[7];
  double h = 0.0, Time = 0.0;
  while (!StopFlag) {
    set_defaults();
    setup_indexing();
    settings.verbose = 0;
    int i;
    auto start = high_resolution_clock::now();
    for (i = 0; i < 7; i++) {
      load_default_data(i);
      load_mpc_params(i, x_old, &h, &Time);
      if (Time >= T)
        break;
      solve();
      x[i].q = vars.x_1[0];
      x[i].v = vars.x_1[1];
      x[i].a = vars.x_1[2];
      if (i == 0)
        fprintf(mpc_fp, "%f %f", Time, h);
      fprintf(mpc_fp, " %f", x[i].q);
    }
    if (Time <= T)
      fprintf(mpc_fp, "\n");
    send_ctrl_params(x, x_old, Time, h);
    auto stop = high_resolution_clock::now();
    auto duration = duration_cast<microseconds>(stop - start);
    usleep(max(0, int(10000 - duration.count())));
    std::cout << duration.count()/1000.0 << std::endl;
    maxTime = max(maxTime, duration.count() / 1000.0);
  }
  pthread_exit(NULL);
}

int main(int argc, char** argv) {
  double err = 0.0;
  mpc_fp = fopen("MPC_DataRecord.txt", "w+");
  ctrl_fp = fopen("Control_DataRecord.txt", "w+");
  printf("%d\n",ctrl_fp);
  msg_initialization();
  pthread_t ctrl_t, mpc_t;
  pthread_mutex_init(&mtx, NULL);
  pthread_create(&ctrl_t, NULL, control_thread, NULL);
  pthread_create(&mpc_t, NULL, MPC_thread, NULL);
  while (!StopFlag) {
    usleep(1000);
  }
  printf("------------DONE-------------\n");
  for (int i = 0; i < 7; i++) {
    if (fabs(q_final[i] - reference[i]) < 1e-6)
      err = 0;
    else
      err = (q_final[i] - reference[i]) / reference[i] * 100;
    printf("Joint %d:\n  Reference:%0.5f deg, Result:%0.5f deg, Error:%0.2f %%\n", i,
           reference[i] / M_PI * 180, q_final[i] / M_PI * 180, err);
  }
  printf("Max MPC calculation time is %0.4fms \n", maxTime);
  pthread_exit(&ctrl_t);
  pthread_exit(&mpc_t);
  pthread_mutex_destroy(&mtx);
  fclose(mpc_fp);
  fclose(ctrl_fp);

  return 0;
}

// Load weight matrix Q & R
// Load constraints
void load_default_data(int joint) {
  /* Make this a diagonal PSD matrix, even though it's not diagonal. */
  params.Q[0] = 0.0;
  params.Q[3] = 0.0;
  params.Q[6] = 0.0;
  params.Q[1] = 0.0;
  params.Q[4] = 1.0;
  params.Q[7] = 0.0;
  params.Q[2] = 0.0;
  params.Q[5] = 0.0;
  params.Q[8] = 1.0;
  /* Make this a diagonal PSD matrix, even though it's not diagonal. */
  params.R[0] = 0.001;
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
  params.rf[1] = 0.0;
  params.rf[2] = 0.0;
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

void msg_initialization() {
  std::cout << "WARNING: This example will move the robot! " << std::endl
            << "Please make sure to have the user stop button at hand!" << std::endl
            << "MOVE the robot to any target position and Press Enter to continue..." << std::endl;
  std::cin.ignore();
  franka::RobotState state = robot.readOnce();
  for (int i = 0; i < 7; i++) {
    reference[i] = state.q[i];
    ;
  }
  for (int i = 0; i < 7; i++) {
    mpc_msg.rf[i] = reference[i];
    mpc_msg.x[i].q = initial_pose[i];
    mpc_msg.x[i].v = 0.0;
    mpc_msg.x[i].a = 0.0;
  }
}

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

// Calculate the discrete transfer matrix
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
