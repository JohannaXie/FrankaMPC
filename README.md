# File system

* doc
* model
* report: Texfile of final report
* schematics
* src
  * cvxgen: Codes for cvxgen
  * data: experiment data record
  * examples: Source codes for project.
  * saved_runs: previous version of source codes
  * simulation: matlab simulation for MPC
  * matlab's `.m` file: Data process to generate figures

# Code

## CVXGEN

The CVXGEN is used to solve the optimization problem of MPC. If a new CVXGEN code is generated, you should use `make` to generate new .o files in the cvxgen folder. Then, copy the following folder to the source file of libfranka:

- util.o, util. c
- solver.c, solver.h, solver.o
- ldl.c, ldl.o
- matrix_support.c, matrix_support.o

Notice: Do rewrite the solver.h for your own program! If you are programming with C++, the function in solver.c should be extended to C++ with `extern "C"`.

## libfranka

Libfranka is already installed on the PC of robot lab connected to Panda. 

### 	Codes

All the source codes in `src/examples` should be copied to `libfranka/examples` before running the program. The final program is `0_MPC_velocity_openloop.cpp`.  If any new programs are added to be compiled, you need to modify the `CMakeLists.txt`.  

### 	Compiling

To compile the codes, run `make` under the folder `libfranka\build` or `libfranka\build\examples`. 

### 	Run

You can run the program under the directory above in the terminal typing `./FileName`. The IP of the robot is `172.16.0.2`, which can be neglected when using our program. But it is required to run the default programs of Panda.