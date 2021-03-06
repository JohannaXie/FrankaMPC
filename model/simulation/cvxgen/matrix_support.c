/* Produced by CVXGEN, 2019-12-10 11:49:06 -0500.  */
/* CVXGEN is Copyright (C) 2006-2017 Jacob Mattingley, jem@cvxgen.com. */
/* The code in this file is Copyright (C) 2006-2017 Jacob Mattingley. */
/* CVXGEN, or solvers produced by CVXGEN, cannot be used for commercial */
/* applications without prior written permission from Jacob Mattingley. */

/* Filename: matrix_support.c. */
/* Description: Support functions for matrix multiplication and vector filling. */
#include "solver.h"
void multbymA(double *lhs, double *rhs) {
  lhs[0] = -rhs[10]*(-params.B[0])-rhs[50]*(-params.A[0])-rhs[51]*(-params.A[3])-rhs[52]*(-params.A[6])-rhs[53]*(1);
  lhs[1] = -rhs[10]*(-params.B[1])-rhs[50]*(-params.A[1])-rhs[51]*(-params.A[4])-rhs[52]*(-params.A[7])-rhs[54]*(1);
  lhs[2] = -rhs[10]*(-params.B[2])-rhs[50]*(-params.A[2])-rhs[51]*(-params.A[5])-rhs[52]*(-params.A[8])-rhs[55]*(1);
  lhs[3] = -rhs[11]*(-params.B[0])-rhs[53]*(-params.A[0])-rhs[54]*(-params.A[3])-rhs[55]*(-params.A[6])-rhs[56]*(1);
  lhs[4] = -rhs[11]*(-params.B[1])-rhs[53]*(-params.A[1])-rhs[54]*(-params.A[4])-rhs[55]*(-params.A[7])-rhs[57]*(1);
  lhs[5] = -rhs[11]*(-params.B[2])-rhs[53]*(-params.A[2])-rhs[54]*(-params.A[5])-rhs[55]*(-params.A[8])-rhs[58]*(1);
  lhs[6] = -rhs[12]*(-params.B[0])-rhs[56]*(-params.A[0])-rhs[57]*(-params.A[3])-rhs[58]*(-params.A[6])-rhs[59]*(1);
  lhs[7] = -rhs[12]*(-params.B[1])-rhs[56]*(-params.A[1])-rhs[57]*(-params.A[4])-rhs[58]*(-params.A[7])-rhs[60]*(1);
  lhs[8] = -rhs[12]*(-params.B[2])-rhs[56]*(-params.A[2])-rhs[57]*(-params.A[5])-rhs[58]*(-params.A[8])-rhs[61]*(1);
  lhs[9] = -rhs[13]*(-params.B[0])-rhs[59]*(-params.A[0])-rhs[60]*(-params.A[3])-rhs[61]*(-params.A[6])-rhs[62]*(1);
  lhs[10] = -rhs[13]*(-params.B[1])-rhs[59]*(-params.A[1])-rhs[60]*(-params.A[4])-rhs[61]*(-params.A[7])-rhs[63]*(1);
  lhs[11] = -rhs[13]*(-params.B[2])-rhs[59]*(-params.A[2])-rhs[60]*(-params.A[5])-rhs[61]*(-params.A[8])-rhs[64]*(1);
  lhs[12] = -rhs[14]*(-params.B[0])-rhs[62]*(-params.A[0])-rhs[63]*(-params.A[3])-rhs[64]*(-params.A[6])-rhs[65]*(1);
  lhs[13] = -rhs[14]*(-params.B[1])-rhs[62]*(-params.A[1])-rhs[63]*(-params.A[4])-rhs[64]*(-params.A[7])-rhs[66]*(1);
  lhs[14] = -rhs[14]*(-params.B[2])-rhs[62]*(-params.A[2])-rhs[63]*(-params.A[5])-rhs[64]*(-params.A[8])-rhs[67]*(1);
  lhs[15] = -rhs[15]*(-params.B[0])-rhs[65]*(-params.A[0])-rhs[66]*(-params.A[3])-rhs[67]*(-params.A[6])-rhs[68]*(1);
  lhs[16] = -rhs[15]*(-params.B[1])-rhs[65]*(-params.A[1])-rhs[66]*(-params.A[4])-rhs[67]*(-params.A[7])-rhs[69]*(1);
  lhs[17] = -rhs[15]*(-params.B[2])-rhs[65]*(-params.A[2])-rhs[66]*(-params.A[5])-rhs[67]*(-params.A[8])-rhs[70]*(1);
  lhs[18] = -rhs[16]*(-params.B[0])-rhs[68]*(-params.A[0])-rhs[69]*(-params.A[3])-rhs[70]*(-params.A[6])-rhs[71]*(1);
  lhs[19] = -rhs[16]*(-params.B[1])-rhs[68]*(-params.A[1])-rhs[69]*(-params.A[4])-rhs[70]*(-params.A[7])-rhs[72]*(1);
  lhs[20] = -rhs[16]*(-params.B[2])-rhs[68]*(-params.A[2])-rhs[69]*(-params.A[5])-rhs[70]*(-params.A[8])-rhs[73]*(1);
  lhs[21] = -rhs[17]*(-params.B[0])-rhs[71]*(-params.A[0])-rhs[72]*(-params.A[3])-rhs[73]*(-params.A[6])-rhs[74]*(1);
  lhs[22] = -rhs[17]*(-params.B[1])-rhs[71]*(-params.A[1])-rhs[72]*(-params.A[4])-rhs[73]*(-params.A[7])-rhs[75]*(1);
  lhs[23] = -rhs[17]*(-params.B[2])-rhs[71]*(-params.A[2])-rhs[72]*(-params.A[5])-rhs[73]*(-params.A[8])-rhs[76]*(1);
  lhs[24] = -rhs[18]*(-params.B[0])-rhs[74]*(-params.A[0])-rhs[75]*(-params.A[3])-rhs[76]*(-params.A[6])-rhs[77]*(1);
  lhs[25] = -rhs[18]*(-params.B[1])-rhs[74]*(-params.A[1])-rhs[75]*(-params.A[4])-rhs[76]*(-params.A[7])-rhs[78]*(1);
  lhs[26] = -rhs[18]*(-params.B[2])-rhs[74]*(-params.A[2])-rhs[75]*(-params.A[5])-rhs[76]*(-params.A[8])-rhs[79]*(1);
  lhs[27] = -rhs[19]*(-params.B[0])-rhs[77]*(-params.A[0])-rhs[78]*(-params.A[3])-rhs[79]*(-params.A[6])-rhs[80]*(1);
  lhs[28] = -rhs[19]*(-params.B[1])-rhs[77]*(-params.A[1])-rhs[78]*(-params.A[4])-rhs[79]*(-params.A[7])-rhs[81]*(1);
  lhs[29] = -rhs[19]*(-params.B[2])-rhs[77]*(-params.A[2])-rhs[78]*(-params.A[5])-rhs[79]*(-params.A[8])-rhs[82]*(1);
  lhs[30] = -rhs[47]*(1);
  lhs[31] = -rhs[48]*(1);
  lhs[32] = -rhs[49]*(1);
  lhs[33] = -rhs[10]*(-params.Gamma_1h[0])-rhs[50]*(-1);
  lhs[34] = -rhs[10]*(-params.Gamma_1h[1])-rhs[51]*(-1);
  lhs[35] = -rhs[10]*(-params.Gamma_1h[2])-rhs[52]*(-1);
  lhs[36] = -rhs[11]*(-params.Gamma_1h[0])-rhs[20]*(1)-rhs[53]*(-1);
  lhs[37] = -rhs[11]*(-params.Gamma_1h[1])-rhs[21]*(1)-rhs[54]*(-1);
  lhs[38] = -rhs[11]*(-params.Gamma_1h[2])-rhs[22]*(1)-rhs[55]*(-1);
  lhs[39] = -rhs[12]*(-params.Gamma_1h[0])-rhs[23]*(1)-rhs[56]*(-1);
  lhs[40] = -rhs[12]*(-params.Gamma_1h[1])-rhs[24]*(1)-rhs[57]*(-1);
  lhs[41] = -rhs[12]*(-params.Gamma_1h[2])-rhs[25]*(1)-rhs[58]*(-1);
  lhs[42] = -rhs[13]*(-params.Gamma_1h[0])-rhs[26]*(1)-rhs[59]*(-1);
  lhs[43] = -rhs[13]*(-params.Gamma_1h[1])-rhs[27]*(1)-rhs[60]*(-1);
  lhs[44] = -rhs[13]*(-params.Gamma_1h[2])-rhs[28]*(1)-rhs[61]*(-1);
  lhs[45] = -rhs[14]*(-params.Gamma_1h[0])-rhs[29]*(1)-rhs[62]*(-1);
  lhs[46] = -rhs[14]*(-params.Gamma_1h[1])-rhs[30]*(1)-rhs[63]*(-1);
  lhs[47] = -rhs[14]*(-params.Gamma_1h[2])-rhs[31]*(1)-rhs[64]*(-1);
  lhs[48] = -rhs[15]*(-params.Gamma_1h[0])-rhs[32]*(1)-rhs[65]*(-1);
  lhs[49] = -rhs[15]*(-params.Gamma_1h[1])-rhs[33]*(1)-rhs[66]*(-1);
  lhs[50] = -rhs[15]*(-params.Gamma_1h[2])-rhs[34]*(1)-rhs[67]*(-1);
  lhs[51] = -rhs[16]*(-params.Gamma_1h[0])-rhs[35]*(1)-rhs[68]*(-1);
  lhs[52] = -rhs[16]*(-params.Gamma_1h[1])-rhs[36]*(1)-rhs[69]*(-1);
  lhs[53] = -rhs[16]*(-params.Gamma_1h[2])-rhs[37]*(1)-rhs[70]*(-1);
  lhs[54] = -rhs[17]*(-params.Gamma_1h[0])-rhs[38]*(1)-rhs[71]*(-1);
  lhs[55] = -rhs[17]*(-params.Gamma_1h[1])-rhs[39]*(1)-rhs[72]*(-1);
  lhs[56] = -rhs[17]*(-params.Gamma_1h[2])-rhs[40]*(1)-rhs[73]*(-1);
  lhs[57] = -rhs[18]*(-params.Gamma_1h[0])-rhs[41]*(1)-rhs[74]*(-1);
  lhs[58] = -rhs[18]*(-params.Gamma_1h[1])-rhs[42]*(1)-rhs[75]*(-1);
  lhs[59] = -rhs[18]*(-params.Gamma_1h[2])-rhs[43]*(1)-rhs[76]*(-1);
  lhs[60] = -rhs[19]*(-params.Gamma_1h[0])-rhs[44]*(1)-rhs[77]*(-1);
  lhs[61] = -rhs[19]*(-params.Gamma_1h[1])-rhs[45]*(1)-rhs[78]*(-1);
  lhs[62] = -rhs[19]*(-params.Gamma_1h[2])-rhs[46]*(1)-rhs[79]*(-1);
  lhs[63] = -rhs[47]*(1)-rhs[80]*(-1);
  lhs[64] = -rhs[48]*(1)-rhs[81]*(-1);
  lhs[65] = -rhs[49]*(1)-rhs[82]*(-1);
}
void multbymAT(double *lhs, double *rhs) {
  lhs[0] = 0;
  lhs[1] = 0;
  lhs[2] = 0;
  lhs[3] = 0;
  lhs[4] = 0;
  lhs[5] = 0;
  lhs[6] = 0;
  lhs[7] = 0;
  lhs[8] = 0;
  lhs[9] = 0;
  lhs[10] = -rhs[0]*(-params.B[0])-rhs[1]*(-params.B[1])-rhs[2]*(-params.B[2])-rhs[33]*(-params.Gamma_1h[0])-rhs[34]*(-params.Gamma_1h[1])-rhs[35]*(-params.Gamma_1h[2]);
  lhs[11] = -rhs[3]*(-params.B[0])-rhs[4]*(-params.B[1])-rhs[5]*(-params.B[2])-rhs[36]*(-params.Gamma_1h[0])-rhs[37]*(-params.Gamma_1h[1])-rhs[38]*(-params.Gamma_1h[2]);
  lhs[12] = -rhs[6]*(-params.B[0])-rhs[7]*(-params.B[1])-rhs[8]*(-params.B[2])-rhs[39]*(-params.Gamma_1h[0])-rhs[40]*(-params.Gamma_1h[1])-rhs[41]*(-params.Gamma_1h[2]);
  lhs[13] = -rhs[9]*(-params.B[0])-rhs[10]*(-params.B[1])-rhs[11]*(-params.B[2])-rhs[42]*(-params.Gamma_1h[0])-rhs[43]*(-params.Gamma_1h[1])-rhs[44]*(-params.Gamma_1h[2]);
  lhs[14] = -rhs[12]*(-params.B[0])-rhs[13]*(-params.B[1])-rhs[14]*(-params.B[2])-rhs[45]*(-params.Gamma_1h[0])-rhs[46]*(-params.Gamma_1h[1])-rhs[47]*(-params.Gamma_1h[2]);
  lhs[15] = -rhs[15]*(-params.B[0])-rhs[16]*(-params.B[1])-rhs[17]*(-params.B[2])-rhs[48]*(-params.Gamma_1h[0])-rhs[49]*(-params.Gamma_1h[1])-rhs[50]*(-params.Gamma_1h[2]);
  lhs[16] = -rhs[18]*(-params.B[0])-rhs[19]*(-params.B[1])-rhs[20]*(-params.B[2])-rhs[51]*(-params.Gamma_1h[0])-rhs[52]*(-params.Gamma_1h[1])-rhs[53]*(-params.Gamma_1h[2]);
  lhs[17] = -rhs[21]*(-params.B[0])-rhs[22]*(-params.B[1])-rhs[23]*(-params.B[2])-rhs[54]*(-params.Gamma_1h[0])-rhs[55]*(-params.Gamma_1h[1])-rhs[56]*(-params.Gamma_1h[2]);
  lhs[18] = -rhs[24]*(-params.B[0])-rhs[25]*(-params.B[1])-rhs[26]*(-params.B[2])-rhs[57]*(-params.Gamma_1h[0])-rhs[58]*(-params.Gamma_1h[1])-rhs[59]*(-params.Gamma_1h[2]);
  lhs[19] = -rhs[27]*(-params.B[0])-rhs[28]*(-params.B[1])-rhs[29]*(-params.B[2])-rhs[60]*(-params.Gamma_1h[0])-rhs[61]*(-params.Gamma_1h[1])-rhs[62]*(-params.Gamma_1h[2]);
  lhs[20] = -rhs[36]*(1);
  lhs[21] = -rhs[37]*(1);
  lhs[22] = -rhs[38]*(1);
  lhs[23] = -rhs[39]*(1);
  lhs[24] = -rhs[40]*(1);
  lhs[25] = -rhs[41]*(1);
  lhs[26] = -rhs[42]*(1);
  lhs[27] = -rhs[43]*(1);
  lhs[28] = -rhs[44]*(1);
  lhs[29] = -rhs[45]*(1);
  lhs[30] = -rhs[46]*(1);
  lhs[31] = -rhs[47]*(1);
  lhs[32] = -rhs[48]*(1);
  lhs[33] = -rhs[49]*(1);
  lhs[34] = -rhs[50]*(1);
  lhs[35] = -rhs[51]*(1);
  lhs[36] = -rhs[52]*(1);
  lhs[37] = -rhs[53]*(1);
  lhs[38] = -rhs[54]*(1);
  lhs[39] = -rhs[55]*(1);
  lhs[40] = -rhs[56]*(1);
  lhs[41] = -rhs[57]*(1);
  lhs[42] = -rhs[58]*(1);
  lhs[43] = -rhs[59]*(1);
  lhs[44] = -rhs[60]*(1);
  lhs[45] = -rhs[61]*(1);
  lhs[46] = -rhs[62]*(1);
  lhs[47] = -rhs[30]*(1)-rhs[63]*(1);
  lhs[48] = -rhs[31]*(1)-rhs[64]*(1);
  lhs[49] = -rhs[32]*(1)-rhs[65]*(1);
  lhs[50] = -rhs[0]*(-params.A[0])-rhs[1]*(-params.A[1])-rhs[2]*(-params.A[2])-rhs[33]*(-1);
  lhs[51] = -rhs[0]*(-params.A[3])-rhs[1]*(-params.A[4])-rhs[2]*(-params.A[5])-rhs[34]*(-1);
  lhs[52] = -rhs[0]*(-params.A[6])-rhs[1]*(-params.A[7])-rhs[2]*(-params.A[8])-rhs[35]*(-1);
  lhs[53] = -rhs[0]*(1)-rhs[3]*(-params.A[0])-rhs[4]*(-params.A[1])-rhs[5]*(-params.A[2])-rhs[36]*(-1);
  lhs[54] = -rhs[1]*(1)-rhs[3]*(-params.A[3])-rhs[4]*(-params.A[4])-rhs[5]*(-params.A[5])-rhs[37]*(-1);
  lhs[55] = -rhs[2]*(1)-rhs[3]*(-params.A[6])-rhs[4]*(-params.A[7])-rhs[5]*(-params.A[8])-rhs[38]*(-1);
  lhs[56] = -rhs[3]*(1)-rhs[6]*(-params.A[0])-rhs[7]*(-params.A[1])-rhs[8]*(-params.A[2])-rhs[39]*(-1);
  lhs[57] = -rhs[4]*(1)-rhs[6]*(-params.A[3])-rhs[7]*(-params.A[4])-rhs[8]*(-params.A[5])-rhs[40]*(-1);
  lhs[58] = -rhs[5]*(1)-rhs[6]*(-params.A[6])-rhs[7]*(-params.A[7])-rhs[8]*(-params.A[8])-rhs[41]*(-1);
  lhs[59] = -rhs[6]*(1)-rhs[9]*(-params.A[0])-rhs[10]*(-params.A[1])-rhs[11]*(-params.A[2])-rhs[42]*(-1);
  lhs[60] = -rhs[7]*(1)-rhs[9]*(-params.A[3])-rhs[10]*(-params.A[4])-rhs[11]*(-params.A[5])-rhs[43]*(-1);
  lhs[61] = -rhs[8]*(1)-rhs[9]*(-params.A[6])-rhs[10]*(-params.A[7])-rhs[11]*(-params.A[8])-rhs[44]*(-1);
  lhs[62] = -rhs[9]*(1)-rhs[12]*(-params.A[0])-rhs[13]*(-params.A[1])-rhs[14]*(-params.A[2])-rhs[45]*(-1);
  lhs[63] = -rhs[10]*(1)-rhs[12]*(-params.A[3])-rhs[13]*(-params.A[4])-rhs[14]*(-params.A[5])-rhs[46]*(-1);
  lhs[64] = -rhs[11]*(1)-rhs[12]*(-params.A[6])-rhs[13]*(-params.A[7])-rhs[14]*(-params.A[8])-rhs[47]*(-1);
  lhs[65] = -rhs[12]*(1)-rhs[15]*(-params.A[0])-rhs[16]*(-params.A[1])-rhs[17]*(-params.A[2])-rhs[48]*(-1);
  lhs[66] = -rhs[13]*(1)-rhs[15]*(-params.A[3])-rhs[16]*(-params.A[4])-rhs[17]*(-params.A[5])-rhs[49]*(-1);
  lhs[67] = -rhs[14]*(1)-rhs[15]*(-params.A[6])-rhs[16]*(-params.A[7])-rhs[17]*(-params.A[8])-rhs[50]*(-1);
  lhs[68] = -rhs[15]*(1)-rhs[18]*(-params.A[0])-rhs[19]*(-params.A[1])-rhs[20]*(-params.A[2])-rhs[51]*(-1);
  lhs[69] = -rhs[16]*(1)-rhs[18]*(-params.A[3])-rhs[19]*(-params.A[4])-rhs[20]*(-params.A[5])-rhs[52]*(-1);
  lhs[70] = -rhs[17]*(1)-rhs[18]*(-params.A[6])-rhs[19]*(-params.A[7])-rhs[20]*(-params.A[8])-rhs[53]*(-1);
  lhs[71] = -rhs[18]*(1)-rhs[21]*(-params.A[0])-rhs[22]*(-params.A[1])-rhs[23]*(-params.A[2])-rhs[54]*(-1);
  lhs[72] = -rhs[19]*(1)-rhs[21]*(-params.A[3])-rhs[22]*(-params.A[4])-rhs[23]*(-params.A[5])-rhs[55]*(-1);
  lhs[73] = -rhs[20]*(1)-rhs[21]*(-params.A[6])-rhs[22]*(-params.A[7])-rhs[23]*(-params.A[8])-rhs[56]*(-1);
  lhs[74] = -rhs[21]*(1)-rhs[24]*(-params.A[0])-rhs[25]*(-params.A[1])-rhs[26]*(-params.A[2])-rhs[57]*(-1);
  lhs[75] = -rhs[22]*(1)-rhs[24]*(-params.A[3])-rhs[25]*(-params.A[4])-rhs[26]*(-params.A[5])-rhs[58]*(-1);
  lhs[76] = -rhs[23]*(1)-rhs[24]*(-params.A[6])-rhs[25]*(-params.A[7])-rhs[26]*(-params.A[8])-rhs[59]*(-1);
  lhs[77] = -rhs[24]*(1)-rhs[27]*(-params.A[0])-rhs[28]*(-params.A[1])-rhs[29]*(-params.A[2])-rhs[60]*(-1);
  lhs[78] = -rhs[25]*(1)-rhs[27]*(-params.A[3])-rhs[28]*(-params.A[4])-rhs[29]*(-params.A[5])-rhs[61]*(-1);
  lhs[79] = -rhs[26]*(1)-rhs[27]*(-params.A[6])-rhs[28]*(-params.A[7])-rhs[29]*(-params.A[8])-rhs[62]*(-1);
  lhs[80] = -rhs[27]*(1)-rhs[63]*(-1);
  lhs[81] = -rhs[28]*(1)-rhs[64]*(-1);
  lhs[82] = -rhs[29]*(1)-rhs[65]*(-1);
}
void multbymG(double *lhs, double *rhs) {
  lhs[0] = -rhs[0]*(1);
  lhs[1] = -rhs[0]*(-1)-rhs[10]*(1);
  lhs[2] = -rhs[0]*(-1)-rhs[10]*(-1);
  lhs[3] = -rhs[1]*(1);
  lhs[4] = -rhs[1]*(-1)-rhs[11]*(1);
  lhs[5] = -rhs[1]*(-1)-rhs[11]*(-1);
  lhs[6] = -rhs[2]*(1);
  lhs[7] = -rhs[2]*(-1)-rhs[12]*(1);
  lhs[8] = -rhs[2]*(-1)-rhs[12]*(-1);
  lhs[9] = -rhs[3]*(1);
  lhs[10] = -rhs[3]*(-1)-rhs[13]*(1);
  lhs[11] = -rhs[3]*(-1)-rhs[13]*(-1);
  lhs[12] = -rhs[4]*(1);
  lhs[13] = -rhs[4]*(-1)-rhs[14]*(1);
  lhs[14] = -rhs[4]*(-1)-rhs[14]*(-1);
  lhs[15] = -rhs[5]*(1);
  lhs[16] = -rhs[5]*(-1)-rhs[15]*(1);
  lhs[17] = -rhs[5]*(-1)-rhs[15]*(-1);
  lhs[18] = -rhs[6]*(1);
  lhs[19] = -rhs[6]*(-1)-rhs[16]*(1);
  lhs[20] = -rhs[6]*(-1)-rhs[16]*(-1);
  lhs[21] = -rhs[7]*(1);
  lhs[22] = -rhs[7]*(-1)-rhs[17]*(1);
  lhs[23] = -rhs[7]*(-1)-rhs[17]*(-1);
  lhs[24] = -rhs[8]*(1);
  lhs[25] = -rhs[8]*(-1)-rhs[18]*(1);
  lhs[26] = -rhs[8]*(-1)-rhs[18]*(-1);
  lhs[27] = -rhs[9]*(1);
  lhs[28] = -rhs[9]*(-1)-rhs[19]*(1);
  lhs[29] = -rhs[9]*(-1)-rhs[19]*(-1);
  lhs[30] = -rhs[20]*(1);
  lhs[31] = -rhs[21]*(1);
  lhs[32] = -rhs[22]*(1);
  lhs[33] = -rhs[23]*(1);
  lhs[34] = -rhs[24]*(1);
  lhs[35] = -rhs[25]*(1);
  lhs[36] = -rhs[26]*(1);
  lhs[37] = -rhs[27]*(1);
  lhs[38] = -rhs[28]*(1);
  lhs[39] = -rhs[29]*(1);
  lhs[40] = -rhs[30]*(1);
  lhs[41] = -rhs[31]*(1);
  lhs[42] = -rhs[32]*(1);
  lhs[43] = -rhs[33]*(1);
  lhs[44] = -rhs[34]*(1);
  lhs[45] = -rhs[35]*(1);
  lhs[46] = -rhs[36]*(1);
  lhs[47] = -rhs[37]*(1);
  lhs[48] = -rhs[38]*(1);
  lhs[49] = -rhs[39]*(1);
  lhs[50] = -rhs[40]*(1);
  lhs[51] = -rhs[41]*(1);
  lhs[52] = -rhs[42]*(1);
  lhs[53] = -rhs[43]*(1);
  lhs[54] = -rhs[44]*(1);
  lhs[55] = -rhs[45]*(1);
  lhs[56] = -rhs[46]*(1);
  lhs[57] = -rhs[47]*(1);
  lhs[58] = -rhs[48]*(1);
  lhs[59] = -rhs[49]*(1);
  lhs[60] = -rhs[20]*(-1);
  lhs[61] = -rhs[21]*(-1);
  lhs[62] = -rhs[22]*(-1);
  lhs[63] = -rhs[23]*(-1);
  lhs[64] = -rhs[24]*(-1);
  lhs[65] = -rhs[25]*(-1);
  lhs[66] = -rhs[26]*(-1);
  lhs[67] = -rhs[27]*(-1);
  lhs[68] = -rhs[28]*(-1);
  lhs[69] = -rhs[29]*(-1);
  lhs[70] = -rhs[30]*(-1);
  lhs[71] = -rhs[31]*(-1);
  lhs[72] = -rhs[32]*(-1);
  lhs[73] = -rhs[33]*(-1);
  lhs[74] = -rhs[34]*(-1);
  lhs[75] = -rhs[35]*(-1);
  lhs[76] = -rhs[36]*(-1);
  lhs[77] = -rhs[37]*(-1);
  lhs[78] = -rhs[38]*(-1);
  lhs[79] = -rhs[39]*(-1);
  lhs[80] = -rhs[40]*(-1);
  lhs[81] = -rhs[41]*(-1);
  lhs[82] = -rhs[42]*(-1);
  lhs[83] = -rhs[43]*(-1);
  lhs[84] = -rhs[44]*(-1);
  lhs[85] = -rhs[45]*(-1);
  lhs[86] = -rhs[46]*(-1);
  lhs[87] = -rhs[47]*(-1);
  lhs[88] = -rhs[48]*(-1);
  lhs[89] = -rhs[49]*(-1);
}
void multbymGT(double *lhs, double *rhs) {
  lhs[0] = -rhs[0]*(1)-rhs[1]*(-1)-rhs[2]*(-1);
  lhs[1] = -rhs[3]*(1)-rhs[4]*(-1)-rhs[5]*(-1);
  lhs[2] = -rhs[6]*(1)-rhs[7]*(-1)-rhs[8]*(-1);
  lhs[3] = -rhs[9]*(1)-rhs[10]*(-1)-rhs[11]*(-1);
  lhs[4] = -rhs[12]*(1)-rhs[13]*(-1)-rhs[14]*(-1);
  lhs[5] = -rhs[15]*(1)-rhs[16]*(-1)-rhs[17]*(-1);
  lhs[6] = -rhs[18]*(1)-rhs[19]*(-1)-rhs[20]*(-1);
  lhs[7] = -rhs[21]*(1)-rhs[22]*(-1)-rhs[23]*(-1);
  lhs[8] = -rhs[24]*(1)-rhs[25]*(-1)-rhs[26]*(-1);
  lhs[9] = -rhs[27]*(1)-rhs[28]*(-1)-rhs[29]*(-1);
  lhs[10] = -rhs[1]*(1)-rhs[2]*(-1);
  lhs[11] = -rhs[4]*(1)-rhs[5]*(-1);
  lhs[12] = -rhs[7]*(1)-rhs[8]*(-1);
  lhs[13] = -rhs[10]*(1)-rhs[11]*(-1);
  lhs[14] = -rhs[13]*(1)-rhs[14]*(-1);
  lhs[15] = -rhs[16]*(1)-rhs[17]*(-1);
  lhs[16] = -rhs[19]*(1)-rhs[20]*(-1);
  lhs[17] = -rhs[22]*(1)-rhs[23]*(-1);
  lhs[18] = -rhs[25]*(1)-rhs[26]*(-1);
  lhs[19] = -rhs[28]*(1)-rhs[29]*(-1);
  lhs[20] = -rhs[30]*(1)-rhs[60]*(-1);
  lhs[21] = -rhs[31]*(1)-rhs[61]*(-1);
  lhs[22] = -rhs[32]*(1)-rhs[62]*(-1);
  lhs[23] = -rhs[33]*(1)-rhs[63]*(-1);
  lhs[24] = -rhs[34]*(1)-rhs[64]*(-1);
  lhs[25] = -rhs[35]*(1)-rhs[65]*(-1);
  lhs[26] = -rhs[36]*(1)-rhs[66]*(-1);
  lhs[27] = -rhs[37]*(1)-rhs[67]*(-1);
  lhs[28] = -rhs[38]*(1)-rhs[68]*(-1);
  lhs[29] = -rhs[39]*(1)-rhs[69]*(-1);
  lhs[30] = -rhs[40]*(1)-rhs[70]*(-1);
  lhs[31] = -rhs[41]*(1)-rhs[71]*(-1);
  lhs[32] = -rhs[42]*(1)-rhs[72]*(-1);
  lhs[33] = -rhs[43]*(1)-rhs[73]*(-1);
  lhs[34] = -rhs[44]*(1)-rhs[74]*(-1);
  lhs[35] = -rhs[45]*(1)-rhs[75]*(-1);
  lhs[36] = -rhs[46]*(1)-rhs[76]*(-1);
  lhs[37] = -rhs[47]*(1)-rhs[77]*(-1);
  lhs[38] = -rhs[48]*(1)-rhs[78]*(-1);
  lhs[39] = -rhs[49]*(1)-rhs[79]*(-1);
  lhs[40] = -rhs[50]*(1)-rhs[80]*(-1);
  lhs[41] = -rhs[51]*(1)-rhs[81]*(-1);
  lhs[42] = -rhs[52]*(1)-rhs[82]*(-1);
  lhs[43] = -rhs[53]*(1)-rhs[83]*(-1);
  lhs[44] = -rhs[54]*(1)-rhs[84]*(-1);
  lhs[45] = -rhs[55]*(1)-rhs[85]*(-1);
  lhs[46] = -rhs[56]*(1)-rhs[86]*(-1);
  lhs[47] = -rhs[57]*(1)-rhs[87]*(-1);
  lhs[48] = -rhs[58]*(1)-rhs[88]*(-1);
  lhs[49] = -rhs[59]*(1)-rhs[89]*(-1);
  lhs[50] = 0;
  lhs[51] = 0;
  lhs[52] = 0;
  lhs[53] = 0;
  lhs[54] = 0;
  lhs[55] = 0;
  lhs[56] = 0;
  lhs[57] = 0;
  lhs[58] = 0;
  lhs[59] = 0;
  lhs[60] = 0;
  lhs[61] = 0;
  lhs[62] = 0;
  lhs[63] = 0;
  lhs[64] = 0;
  lhs[65] = 0;
  lhs[66] = 0;
  lhs[67] = 0;
  lhs[68] = 0;
  lhs[69] = 0;
  lhs[70] = 0;
  lhs[71] = 0;
  lhs[72] = 0;
  lhs[73] = 0;
  lhs[74] = 0;
  lhs[75] = 0;
  lhs[76] = 0;
  lhs[77] = 0;
  lhs[78] = 0;
  lhs[79] = 0;
  lhs[80] = 0;
  lhs[81] = 0;
  lhs[82] = 0;
}
void multbyP(double *lhs, double *rhs) {
  /* TODO use the fact that P is symmetric? */
  /* TODO check doubling / half factor etc. */
  lhs[0] = 0;
  lhs[1] = 0;
  lhs[2] = 0;
  lhs[3] = 0;
  lhs[4] = 0;
  lhs[5] = 0;
  lhs[6] = 0;
  lhs[7] = 0;
  lhs[8] = 0;
  lhs[9] = 0;
  lhs[10] = rhs[10]*(2*params.R[0]);
  lhs[11] = rhs[11]*(2*params.R[0]);
  lhs[12] = rhs[12]*(2*params.R[0]);
  lhs[13] = rhs[13]*(2*params.R[0]);
  lhs[14] = rhs[14]*(2*params.R[0]);
  lhs[15] = rhs[15]*(2*params.R[0]);
  lhs[16] = rhs[16]*(2*params.R[0]);
  lhs[17] = rhs[17]*(2*params.R[0]);
  lhs[18] = rhs[18]*(2*params.R[0]);
  lhs[19] = rhs[19]*(2*params.R[0]);
  lhs[20] = rhs[20]*(2*params.Q[0])+rhs[21]*(2*params.Q[3])+rhs[22]*(2*params.Q[6]);
  lhs[21] = rhs[20]*(2*params.Q[1])+rhs[21]*(2*params.Q[4])+rhs[22]*(2*params.Q[7]);
  lhs[22] = rhs[20]*(2*params.Q[2])+rhs[21]*(2*params.Q[5])+rhs[22]*(2*params.Q[8]);
  lhs[23] = rhs[23]*(2*params.Q[0])+rhs[24]*(2*params.Q[3])+rhs[25]*(2*params.Q[6]);
  lhs[24] = rhs[23]*(2*params.Q[1])+rhs[24]*(2*params.Q[4])+rhs[25]*(2*params.Q[7]);
  lhs[25] = rhs[23]*(2*params.Q[2])+rhs[24]*(2*params.Q[5])+rhs[25]*(2*params.Q[8]);
  lhs[26] = rhs[26]*(2*params.Q[0])+rhs[27]*(2*params.Q[3])+rhs[28]*(2*params.Q[6]);
  lhs[27] = rhs[26]*(2*params.Q[1])+rhs[27]*(2*params.Q[4])+rhs[28]*(2*params.Q[7]);
  lhs[28] = rhs[26]*(2*params.Q[2])+rhs[27]*(2*params.Q[5])+rhs[28]*(2*params.Q[8]);
  lhs[29] = rhs[29]*(2*params.Q[0])+rhs[30]*(2*params.Q[3])+rhs[31]*(2*params.Q[6]);
  lhs[30] = rhs[29]*(2*params.Q[1])+rhs[30]*(2*params.Q[4])+rhs[31]*(2*params.Q[7]);
  lhs[31] = rhs[29]*(2*params.Q[2])+rhs[30]*(2*params.Q[5])+rhs[31]*(2*params.Q[8]);
  lhs[32] = rhs[32]*(2*params.Q[0])+rhs[33]*(2*params.Q[3])+rhs[34]*(2*params.Q[6]);
  lhs[33] = rhs[32]*(2*params.Q[1])+rhs[33]*(2*params.Q[4])+rhs[34]*(2*params.Q[7]);
  lhs[34] = rhs[32]*(2*params.Q[2])+rhs[33]*(2*params.Q[5])+rhs[34]*(2*params.Q[8]);
  lhs[35] = rhs[35]*(2*params.Q[0])+rhs[36]*(2*params.Q[3])+rhs[37]*(2*params.Q[6]);
  lhs[36] = rhs[35]*(2*params.Q[1])+rhs[36]*(2*params.Q[4])+rhs[37]*(2*params.Q[7]);
  lhs[37] = rhs[35]*(2*params.Q[2])+rhs[36]*(2*params.Q[5])+rhs[37]*(2*params.Q[8]);
  lhs[38] = rhs[38]*(2*params.Q[0])+rhs[39]*(2*params.Q[3])+rhs[40]*(2*params.Q[6]);
  lhs[39] = rhs[38]*(2*params.Q[1])+rhs[39]*(2*params.Q[4])+rhs[40]*(2*params.Q[7]);
  lhs[40] = rhs[38]*(2*params.Q[2])+rhs[39]*(2*params.Q[5])+rhs[40]*(2*params.Q[8]);
  lhs[41] = rhs[41]*(2*params.Q[0])+rhs[42]*(2*params.Q[3])+rhs[43]*(2*params.Q[6]);
  lhs[42] = rhs[41]*(2*params.Q[1])+rhs[42]*(2*params.Q[4])+rhs[43]*(2*params.Q[7]);
  lhs[43] = rhs[41]*(2*params.Q[2])+rhs[42]*(2*params.Q[5])+rhs[43]*(2*params.Q[8]);
  lhs[44] = rhs[44]*(2*params.Q[0])+rhs[45]*(2*params.Q[3])+rhs[46]*(2*params.Q[6]);
  lhs[45] = rhs[44]*(2*params.Q[1])+rhs[45]*(2*params.Q[4])+rhs[46]*(2*params.Q[7]);
  lhs[46] = rhs[44]*(2*params.Q[2])+rhs[45]*(2*params.Q[5])+rhs[46]*(2*params.Q[8]);
  lhs[47] = rhs[47]*(2*params.Q[0])+rhs[48]*(2*params.Q[3])+rhs[49]*(2*params.Q[6]);
  lhs[48] = rhs[47]*(2*params.Q[1])+rhs[48]*(2*params.Q[4])+rhs[49]*(2*params.Q[7]);
  lhs[49] = rhs[47]*(2*params.Q[2])+rhs[48]*(2*params.Q[5])+rhs[49]*(2*params.Q[8]);
  lhs[50] = 0;
  lhs[51] = 0;
  lhs[52] = 0;
  lhs[53] = 0;
  lhs[54] = 0;
  lhs[55] = 0;
  lhs[56] = 0;
  lhs[57] = 0;
  lhs[58] = 0;
  lhs[59] = 0;
  lhs[60] = 0;
  lhs[61] = 0;
  lhs[62] = 0;
  lhs[63] = 0;
  lhs[64] = 0;
  lhs[65] = 0;
  lhs[66] = 0;
  lhs[67] = 0;
  lhs[68] = 0;
  lhs[69] = 0;
  lhs[70] = 0;
  lhs[71] = 0;
  lhs[72] = 0;
  lhs[73] = 0;
  lhs[74] = 0;
  lhs[75] = 0;
  lhs[76] = 0;
  lhs[77] = 0;
  lhs[78] = 0;
  lhs[79] = 0;
  lhs[80] = 0;
  lhs[81] = 0;
  lhs[82] = 0;
}
void fillq(void) {
  work.q[0] = 0;
  work.q[1] = 0;
  work.q[2] = 0;
  work.q[3] = 0;
  work.q[4] = 0;
  work.q[5] = 0;
  work.q[6] = 0;
  work.q[7] = 0;
  work.q[8] = 0;
  work.q[9] = 0;
  work.q[10] = 0;
  work.q[11] = 0;
  work.q[12] = 0;
  work.q[13] = 0;
  work.q[14] = 0;
  work.q[15] = 0;
  work.q[16] = 0;
  work.q[17] = 0;
  work.q[18] = 0;
  work.q[19] = 0;
  work.q[20] = 0;
  work.q[21] = 0;
  work.q[22] = 0;
  work.q[23] = 0;
  work.q[24] = 0;
  work.q[25] = 0;
  work.q[26] = 0;
  work.q[27] = 0;
  work.q[28] = 0;
  work.q[29] = 0;
  work.q[30] = 0;
  work.q[31] = 0;
  work.q[32] = 0;
  work.q[33] = 0;
  work.q[34] = 0;
  work.q[35] = 0;
  work.q[36] = 0;
  work.q[37] = 0;
  work.q[38] = 0;
  work.q[39] = 0;
  work.q[40] = 0;
  work.q[41] = 0;
  work.q[42] = 0;
  work.q[43] = 0;
  work.q[44] = 0;
  work.q[45] = 0;
  work.q[46] = 0;
  work.q[47] = 0;
  work.q[48] = 0;
  work.q[49] = 0;
  work.q[50] = 0;
  work.q[51] = 0;
  work.q[52] = 0;
  work.q[53] = 0;
  work.q[54] = 0;
  work.q[55] = 0;
  work.q[56] = 0;
  work.q[57] = 0;
  work.q[58] = 0;
  work.q[59] = 0;
  work.q[60] = 0;
  work.q[61] = 0;
  work.q[62] = 0;
  work.q[63] = 0;
  work.q[64] = 0;
  work.q[65] = 0;
  work.q[66] = 0;
  work.q[67] = 0;
  work.q[68] = 0;
  work.q[69] = 0;
  work.q[70] = 0;
  work.q[71] = 0;
  work.q[72] = 0;
  work.q[73] = 0;
  work.q[74] = 0;
  work.q[75] = 0;
  work.q[76] = 0;
  work.q[77] = 0;
  work.q[78] = 0;
  work.q[79] = 0;
  work.q[80] = 0;
  work.q[81] = 0;
  work.q[82] = 0;
}
void fillh(void) {
  work.h[0] = params.u_max[0];
  work.h[1] = 0;
  work.h[2] = 0;
  work.h[3] = params.u_max[0];
  work.h[4] = 0;
  work.h[5] = 0;
  work.h[6] = params.u_max[0];
  work.h[7] = 0;
  work.h[8] = 0;
  work.h[9] = params.u_max[0];
  work.h[10] = 0;
  work.h[11] = 0;
  work.h[12] = params.u_max[0];
  work.h[13] = 0;
  work.h[14] = 0;
  work.h[15] = params.u_max[0];
  work.h[16] = 0;
  work.h[17] = 0;
  work.h[18] = params.u_max[0];
  work.h[19] = 0;
  work.h[20] = 0;
  work.h[21] = params.u_max[0];
  work.h[22] = 0;
  work.h[23] = 0;
  work.h[24] = params.u_max[0];
  work.h[25] = 0;
  work.h[26] = 0;
  work.h[27] = params.u_max[0];
  work.h[28] = 0;
  work.h[29] = 0;
  work.h[30] = params.x_max[0];
  work.h[31] = params.x_max[1];
  work.h[32] = params.x_max[2];
  work.h[33] = params.x_max[0];
  work.h[34] = params.x_max[1];
  work.h[35] = params.x_max[2];
  work.h[36] = params.x_max[0];
  work.h[37] = params.x_max[1];
  work.h[38] = params.x_max[2];
  work.h[39] = params.x_max[0];
  work.h[40] = params.x_max[1];
  work.h[41] = params.x_max[2];
  work.h[42] = params.x_max[0];
  work.h[43] = params.x_max[1];
  work.h[44] = params.x_max[2];
  work.h[45] = params.x_max[0];
  work.h[46] = params.x_max[1];
  work.h[47] = params.x_max[2];
  work.h[48] = params.x_max[0];
  work.h[49] = params.x_max[1];
  work.h[50] = params.x_max[2];
  work.h[51] = params.x_max[0];
  work.h[52] = params.x_max[1];
  work.h[53] = params.x_max[2];
  work.h[54] = params.x_max[0];
  work.h[55] = params.x_max[1];
  work.h[56] = params.x_max[2];
  work.h[57] = params.x_max[0];
  work.h[58] = params.x_max[1];
  work.h[59] = params.x_max[2];
  work.h[60] = -params.x_min[0];
  work.h[61] = -params.x_min[1];
  work.h[62] = -params.x_min[2];
  work.h[63] = -params.x_min[0];
  work.h[64] = -params.x_min[1];
  work.h[65] = -params.x_min[2];
  work.h[66] = -params.x_min[0];
  work.h[67] = -params.x_min[1];
  work.h[68] = -params.x_min[2];
  work.h[69] = -params.x_min[0];
  work.h[70] = -params.x_min[1];
  work.h[71] = -params.x_min[2];
  work.h[72] = -params.x_min[0];
  work.h[73] = -params.x_min[1];
  work.h[74] = -params.x_min[2];
  work.h[75] = -params.x_min[0];
  work.h[76] = -params.x_min[1];
  work.h[77] = -params.x_min[2];
  work.h[78] = -params.x_min[0];
  work.h[79] = -params.x_min[1];
  work.h[80] = -params.x_min[2];
  work.h[81] = -params.x_min[0];
  work.h[82] = -params.x_min[1];
  work.h[83] = -params.x_min[2];
  work.h[84] = -params.x_min[0];
  work.h[85] = -params.x_min[1];
  work.h[86] = -params.x_min[2];
  work.h[87] = -params.x_min[0];
  work.h[88] = -params.x_min[1];
  work.h[89] = -params.x_min[2];
}
void fillb(void) {
  work.b[0] = 0;
  work.b[1] = 0;
  work.b[2] = 0;
  work.b[3] = 0;
  work.b[4] = 0;
  work.b[5] = 0;
  work.b[6] = 0;
  work.b[7] = 0;
  work.b[8] = 0;
  work.b[9] = 0;
  work.b[10] = 0;
  work.b[11] = 0;
  work.b[12] = 0;
  work.b[13] = 0;
  work.b[14] = 0;
  work.b[15] = 0;
  work.b[16] = 0;
  work.b[17] = 0;
  work.b[18] = 0;
  work.b[19] = 0;
  work.b[20] = 0;
  work.b[21] = 0;
  work.b[22] = 0;
  work.b[23] = 0;
  work.b[24] = 0;
  work.b[25] = 0;
  work.b[26] = 0;
  work.b[27] = 0;
  work.b[28] = 0;
  work.b[29] = 0;
  work.b[30] = params.rf[0];
  work.b[31] = params.rf[1];
  work.b[32] = params.rf[2];
  work.b[33] = -params.x_0[0];
  work.b[34] = -params.x_0[1];
  work.b[35] = -params.x_0[2];
  work.b[36] = 0;
  work.b[37] = 0;
  work.b[38] = 0;
  work.b[39] = 0;
  work.b[40] = 0;
  work.b[41] = 0;
  work.b[42] = 0;
  work.b[43] = 0;
  work.b[44] = 0;
  work.b[45] = 0;
  work.b[46] = 0;
  work.b[47] = 0;
  work.b[48] = 0;
  work.b[49] = 0;
  work.b[50] = 0;
  work.b[51] = 0;
  work.b[52] = 0;
  work.b[53] = 0;
  work.b[54] = 0;
  work.b[55] = 0;
  work.b[56] = 0;
  work.b[57] = 0;
  work.b[58] = 0;
  work.b[59] = 0;
  work.b[60] = 0;
  work.b[61] = 0;
  work.b[62] = 0;
  work.b[63] = 0;
  work.b[64] = 0;
  work.b[65] = 0;
}
void pre_ops(void) {
}
