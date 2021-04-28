
// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <cmath>
#include <iostream>
#include <franka/exception.h>
#include <franka/robot.h>
#include <pthread.h>
#include "examples_common.h"
#include "solver.h"
#include <pthread.h>

/**
 * Author: 
 * 
 * Date
 * @warning Before executing this example, make sure there is enough space in front of the robot.
 */

// Variables definition for CVXGEN

Vars vars;
Params params;
Workspace work;
Settings settings;

double h = 0.1;


int main(int argc, char** argv) 
{

	if (argc != 2) 
	{
		std::cerr << "Usage: " << argv[0] << " <robot-hostname>" << std::endl;
		return -1;
	}
	try 
	{
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
      {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
      {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
      {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}},
      {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}});

    // Create threads

    std::array<double, 7> initial_position;
    double time = 0.0;
    double initial_state;
    int i;
    franka::RobotState robot_state = robot.readOnce();
    initial_state = robot_state.q_d[4];
    // MPC initialization
    set_defaults();
    setup_indexing();
    load_default_data();
    settings.verbose = 1;
    calculate_transfer_matrix(h);
    load_initial_state(initial_state);
    load_target_state(initial_state+M_PI/32);
    printf("Initial state: %9.4f\n Target state: %9.4f\n",robot_state.q_d[4],robot_state.q_d[4]+M_PI/32);
    solve();
    printf("CVXGEN FINISHED\n");
    for (i=0;i<3;i++){
      printf("%9.4f ",vars.x_1[i]);
    }
    printf("\n");
    robot.control([&initial_position, &time](const franka::RobotState& robot_state,
     franka::Duration period) -> franka::JointPositions 
    {
      time += period.toSec();

      if (time == 0.0) 
      { 
        initial_position = robot_state.q_d;
      }

      // double delta_angle = M_PI / 32.0 * (1 - std::cos(M_PI / 2.5 * time));
      double delta_angle = vars.x_1[0]*(time/h);
      franka::JointPositions output = {{initial_position[0], initial_position[1], initial_position[2], initial_position[3],                                        initial_position[4] + delta_angle, 
        initial_position[5], initial_position[6]}};

        if (time >= h) {
          std::cout << std::endl << "Finished motion, shutting down example" << std::endl;
          return franka::MotionFinished(output);
        }
        return output;
      });
  } catch (const franka::Exception& e) {
    std::cout << e.what() << std::endl;
    return -1;
  }

  return 0;
}


// Load weight matrix Q & R
// Load constraints 
void load_default_data(void) {
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

  params.u_max[0] = 7500;
  params.x_max[0] = 2.8973;
  params.x_max[1] = 2.6100;
  params.x_max[2] = 15.0;
  params.x_min[0] = -2.8973;
  params.x_min[1] = -2.6100;
  params.x_min[2] = -15.0;

}

// Load initial state
void load_initial_state(double initial_position){
  params.x_0[0] = initial_position;
  params.x_0[1] = 0.0;
  params.x_0[2] = 0.0;  
}

void load_target_state(double target_position){
  params.rf[0] = target_position;
  params.rf[1] = 0.0;
  params.rf[2] = 0.0;
}

// Calculate the discrete transfer matrix
void calculate_transfer_matrix(double h){
  params.A[0] = 1.0;
  params.A[1] = 0.0;
  params.A[2] = 0.0;
  params.A[3] = h;
  params.A[4] = 1.0;
  params.A[5] = 0.0;
  params.A[6] = h*h*0.5;
  params.A[7] = h;
  params.A[8] = 1.0;

  params.B[0] = 7*h*h*h/12.0;
  params.B[1] = h*h;
  params.B[2] = h;  
}
