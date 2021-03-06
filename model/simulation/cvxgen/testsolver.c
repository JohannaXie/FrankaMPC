/* Produced by CVXGEN, 2019-12-10 11:49:06 -0500.  */
/* CVXGEN is Copyright (C) 2006-2017 Jacob Mattingley, jem@cvxgen.com. */
/* The code in this file is Copyright (C) 2006-2017 Jacob Mattingley. */
/* CVXGEN, or solvers produced by CVXGEN, cannot be used for commercial */
/* applications without prior written permission from Jacob Mattingley. */

/* Filename: testsolver.c. */
/* Description: Basic test harness for solver.c. */
#include "solver.h"
Vars vars;
Params params;
Workspace work;
Settings settings;
#define NUMTESTS 0
int main(int argc, char **argv) {
  int num_iters;
#if (NUMTESTS > 0)
  int i;
  double time;
  double time_per;
#endif
  set_defaults();
  setup_indexing();
  load_default_data();
  /* Solve problem instance for the record. */
  settings.verbose = 1;
  num_iters = solve();
#ifndef ZERO_LIBRARY_MODE
#if (NUMTESTS > 0)
  /* Now solve multiple problem instances for timing purposes. */
  settings.verbose = 0;
  tic();
  for (i = 0; i < NUMTESTS; i++) {
    solve();
  }
  time = tocq();
  printf("Timed %d solves over %.3f seconds.\n", NUMTESTS, time);
  time_per = time / NUMTESTS;
  if (time_per > 1) {
    printf("Actual time taken per solve: %.3g s.\n", time_per);
  } else if (time_per > 1e-3) {
    printf("Actual time taken per solve: %.3g ms.\n", 1e3*time_per);
  } else {
    printf("Actual time taken per solve: %.3g us.\n", 1e6*time_per);
  }
#endif
#endif
  return 0;
}
void load_default_data(void) {
  /* Make this a diagonal PSD matrix, even though it's not diagonal. */
  params.Q[0] = 1.5507979025745755;
  params.Q[3] = 0;
  params.Q[6] = 0;
  params.Q[1] = 0;
  params.Q[4] = 1.7081478226181048;
  params.Q[7] = 0;
  params.Q[2] = 0;
  params.Q[5] = 0;
  params.Q[8] = 1.2909047389129444;
  /* Make this a diagonal PSD matrix, even though it's not diagonal. */
  params.R[0] = 1.510827605197663;
  params.A[0] = 1.5717878173906188;
  params.A[1] = 1.5851723557337523;
  params.A[2] = -1.497658758144655;
  params.A[3] = -1.171028487447253;
  params.A[4] = -1.7941311867966805;
  params.A[5] = -0.23676062539745413;
  params.A[6] = -1.8804951564857322;
  params.A[7] = -0.17266710242115568;
  params.A[8] = 0.596576190459043;
  params.B[0] = -0.8860508694080989;
  params.B[1] = 0.7050196079205251;
  params.B[2] = 0.3634512696654033;
  params.u_max[0] = 0.04796376475433073;
  params.x_max[0] = 1.117708175981764;
  params.x_max[1] = 0.5185048938149308;
  params.x_max[2] = 0.8302023940201393;
  params.x_min[0] = -0.5670501635426375;
  params.x_min[1] = -1.3862758366259926;
  params.x_min[2] = -0.880907435341479;
  params.rf[0] = -1.372529046100147;
  params.rf[1] = 0.17859607212737894;
  params.rf[2] = 1.1212590580454682;
  params.x_0[0] = -0.774545870495281;
  params.x_0[1] = -1.1121684642712744;
  params.x_0[2] = -0.44811496977740495;
  params.Gamma_1h[0] = 1.7455345994417217;
  params.Gamma_1h[1] = 1.9039816898917352;
  params.Gamma_1h[2] = 0.6895347036512547;
}
