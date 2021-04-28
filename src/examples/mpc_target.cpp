#include <cmath>
#include <iostream>
#include <franka/exception.h>
#include <franka/robot.h>
#include <pthread.h>
#include "examples_common.h"
#include "solver.h"
#include <pthread.h>
#include <chrono>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>


Vars vars;
Params params;
Workspace work;
Settings settings;

int main(int argc, char** argv) {
	FILE *mpc_target;	
  try {
    franka::Robot robot("172.16.0.2");

    franka::RobotState state = robot.readOnce();

    mpc_target = fopen("MPC_target.txt","w+");
    for (int i=0;i<7;i++){
    	fprintf(mpc_target,"%f\n",state.q[i]);
    	printf("Joint %d: %f\n",i,state.q[i]/M_PI*180);
    }
    fclose(mpc_target);
    mpc_target = fopen("MPC_target.txt","r");    
    float tmp;
    for (int i=0;i<7;i++){
    	fscanf(mpc_target, "%f\n",&tmp);
		printf("Joint %d: %f\n",i,tmp/M_PI*180);    	
    }
    fclose(mpc_target);
  } catch (franka::Exception const& e) {
    std::cout << e.what() << std::endl;
    return -1;
  }

  return 0;
}
