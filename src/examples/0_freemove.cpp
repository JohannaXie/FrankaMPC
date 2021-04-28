// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <cmath>
#include <iostream>

#include <franka/exception.h>
#include <franka/robot.h>

#include "examples_common.h"

/**
 * @example generate_joint_velocity_motion.cpp
 * An example showing how to generate a joint velocity motion.
 *
 * @warning Before executing this example, make sure there is enough space in front of the robot.
 */

int main(int argc, char** argv) {
  if (argc != 2) {
    std::cerr << "Usage: " << argv[0] << " <robot-hostname>" << std::endl;
    return -1;
  }
  try {
    franka::Robot robot(argv[1]);
    setDefaultBehavior(robot);

    // First move the robot to a suitable joint configuration
    std::array<double, 7> q_goal = {{0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4}};
    MotionGenerator motion_generator(0.5, q_goal);
    std::cout << "WARNING: This example will move the robot! "
              << "Please make sure to have the user stop button at hand!" << std::endl
              << "Press Enter to continue..." << std::endl;
    std::cin.ignore();
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

    double time_max = 60.0;
    double omega_max = 1.0;
    double time = 0.0;

    std::array<double, 7> tau_ext;
    int pert_flag = 0;

    robot.control(
        [=, &time, &tau_ext, &pert_flag](const franka::RobotState& robot_state,
                                         franka::Duration period) -> franka::JointVelocities {
          time += period.toSec();
          tau_ext = robot_state.tau_ext_hat_filtered;
          franka::JointVelocities velocities = {{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
          for (size_t i = 0; i < 7; i++) {
            
            velocities.dq[i] = -0.015 * tau_ext[i];
          }
          std::cout << tau_ext[5] <<std::endl;

          if (time >= 2 * time_max) {
            std::cout << "Finished motion, shutting down example" << std::endl;
            return franka::MotionFinished(velocities);
          }
          return velocities;
        });
  } catch (const franka::Exception& e) {
    std::cout << e.what() << std::endl;
    return -1;
  }

  return 0;
}

void follow() {}
