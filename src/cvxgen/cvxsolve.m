% Produced by CVXGEN, 2019-12-21 11:21:07 -0500.
% CVXGEN is Copyright (C) 2006-2017 Jacob Mattingley, jem@cvxgen.com.
% The code in this file is Copyright (C) 2006-2017 Jacob Mattingley.
% CVXGEN, or solvers produced by CVXGEN, cannot be used for commercial
% applications without prior written permission from Jacob Mattingley.

% Filename: cvxsolve.m.
% Description: Solution file, via cvx, for use with sample.m.
function [vars, status] = cvxsolve(params, settings)
A = params.A;
B = params.B;
Gamma_1h = params.Gamma_1h;
Q = params.Q;
R = params.R;
rf = params.rf;
u_max = params.u_max;
x_0 = params.x_0;
x_max = params.x_max;
x_min = params.x_min;
cvx_begin
  % Caution: automatically generated by cvxgen. May be incorrect.
  variable x_1(3, 1);
  variable x_2(3, 1);
  variable x_3(3, 1);
  variable x_4(3, 1);
  variable x_5(3, 1);
  variable x_6(3, 1);
  variable x_7(3, 1);
  variable x_8(3, 1);
  variable x_9(3, 1);
  variable x_10(3, 1);
  variable x_11(3, 1);
  variable x_12(3, 1);
  variable x_13(3, 1);
  variable x_14(3, 1);
  variable x_15(3, 1);
  variable x_16(3, 1);
  variable x_17(3, 1);
  variable x_18(3, 1);
  variable x_19(3, 1);
  variable x_20(3, 1);
  variable x_21(3, 1);
  variable x_22(3, 1);
  variable x_23(3, 1);
  variable x_24(3, 1);
  variable x_25(3, 1);
  variable u_0;
  variable u_1;
  variable u_2;
  variable u_3;
  variable u_4;
  variable u_5;
  variable u_6;
  variable u_7;
  variable u_8;
  variable u_9;
  variable u_10;
  variable u_11;
  variable u_12;
  variable u_13;
  variable u_14;
  variable u_15;
  variable u_16;
  variable u_17;
  variable u_18;
  variable u_19;
  variable u_20;
  variable u_21;
  variable u_22;
  variable u_23;
  variable u_24;
  variable z_1(3, 1);
  variable z_0(3, 1);
  variable z_2(3, 1);
  variable z_3(3, 1);
  variable z_4(3, 1);
  variable z_5(3, 1);
  variable z_6(3, 1);
  variable z_7(3, 1);
  variable z_8(3, 1);
  variable z_9(3, 1);
  variable z_10(3, 1);
  variable z_11(3, 1);
  variable z_12(3, 1);
  variable z_13(3, 1);
  variable z_14(3, 1);
  variable z_15(3, 1);
  variable z_16(3, 1);
  variable z_17(3, 1);
  variable z_18(3, 1);
  variable z_19(3, 1);
  variable z_20(3, 1);
  variable z_21(3, 1);
  variable z_22(3, 1);
  variable z_23(3, 1);
  variable z_24(3, 1);
  variable z_25(3, 1);

  minimize(quad_form(x_1, Q) + quad_form(x_2, Q) + quad_form(x_3, Q) + quad_form(x_4, Q) + quad_form(x_5, Q) + quad_form(x_6, Q) + quad_form(x_7, Q) + quad_form(x_8, Q) + quad_form(x_9, Q) + quad_form(x_10, Q) + quad_form(x_11, Q) + quad_form(x_12, Q) + quad_form(x_13, Q) + quad_form(x_14, Q) + quad_form(x_15, Q) + quad_form(x_16, Q) + quad_form(x_17, Q) + quad_form(x_18, Q) + quad_form(x_19, Q) + quad_form(x_20, Q) + quad_form(x_21, Q) + quad_form(x_22, Q) + quad_form(x_23, Q) + quad_form(x_24, Q) + quad_form(x_25, Q) + quad_form(u_0, R) + quad_form(u_1, R) + quad_form(u_2, R) + quad_form(u_3, R) + quad_form(u_4, R) + quad_form(u_5, R) + quad_form(u_6, R) + quad_form(u_7, R) + quad_form(u_8, R) + quad_form(u_9, R) + quad_form(u_10, R) + quad_form(u_11, R) + quad_form(u_12, R) + quad_form(u_13, R) + quad_form(u_14, R) + quad_form(u_15, R) + quad_form(u_16, R) + quad_form(u_17, R) + quad_form(u_18, R) + quad_form(u_19, R) + quad_form(u_20, R) + quad_form(u_21, R) + quad_form(u_22, R) + quad_form(u_23, R) + quad_form(u_24, R));
  subject to
    z_1 == A*z_0 + B*u_0;
    z_2 == A*z_1 + B*u_1;
    z_3 == A*z_2 + B*u_2;
    z_4 == A*z_3 + B*u_3;
    z_5 == A*z_4 + B*u_4;
    z_6 == A*z_5 + B*u_5;
    z_7 == A*z_6 + B*u_6;
    z_8 == A*z_7 + B*u_7;
    z_9 == A*z_8 + B*u_8;
    z_10 == A*z_9 + B*u_9;
    z_11 == A*z_10 + B*u_10;
    z_12 == A*z_11 + B*u_11;
    z_13 == A*z_12 + B*u_12;
    z_14 == A*z_13 + B*u_13;
    z_15 == A*z_14 + B*u_14;
    z_16 == A*z_15 + B*u_15;
    z_17 == A*z_16 + B*u_16;
    z_18 == A*z_17 + B*u_17;
    z_19 == A*z_18 + B*u_18;
    z_20 == A*z_19 + B*u_19;
    z_21 == A*z_20 + B*u_20;
    z_22 == A*z_21 + B*u_21;
    z_23 == A*z_22 + B*u_22;
    z_24 == A*z_23 + B*u_23;
    z_25 == A*z_24 + B*u_24;
    abs(u_0) <= u_max;
    abs(u_1) <= u_max;
    abs(u_2) <= u_max;
    abs(u_3) <= u_max;
    abs(u_4) <= u_max;
    abs(u_5) <= u_max;
    abs(u_6) <= u_max;
    abs(u_7) <= u_max;
    abs(u_8) <= u_max;
    abs(u_9) <= u_max;
    abs(u_10) <= u_max;
    abs(u_11) <= u_max;
    abs(u_12) <= u_max;
    abs(u_13) <= u_max;
    abs(u_14) <= u_max;
    abs(u_15) <= u_max;
    abs(u_16) <= u_max;
    abs(u_17) <= u_max;
    abs(u_18) <= u_max;
    abs(u_19) <= u_max;
    abs(u_20) <= u_max;
    abs(u_21) <= u_max;
    abs(u_22) <= u_max;
    abs(u_23) <= u_max;
    abs(u_24) <= u_max;
    x_1 <= x_max;
    x_2 <= x_max;
    x_3 <= x_max;
    x_4 <= x_max;
    x_5 <= x_max;
    x_6 <= x_max;
    x_7 <= x_max;
    x_8 <= x_max;
    x_9 <= x_max;
    x_10 <= x_max;
    x_11 <= x_max;
    x_12 <= x_max;
    x_13 <= x_max;
    x_14 <= x_max;
    x_15 <= x_max;
    x_16 <= x_max;
    x_17 <= x_max;
    x_18 <= x_max;
    x_19 <= x_max;
    x_20 <= x_max;
    x_21 <= x_max;
    x_22 <= x_max;
    x_23 <= x_max;
    x_24 <= x_max;
    x_25 <= x_max;
    x_1 >= x_min;
    x_2 >= x_min;
    x_3 >= x_min;
    x_4 >= x_min;
    x_5 >= x_min;
    x_6 >= x_min;
    x_7 >= x_min;
    x_8 >= x_min;
    x_9 >= x_min;
    x_10 >= x_min;
    x_11 >= x_min;
    x_12 >= x_min;
    x_13 >= x_min;
    x_14 >= x_min;
    x_15 >= x_min;
    x_16 >= x_min;
    x_17 >= x_min;
    x_18 >= x_min;
    x_19 >= x_min;
    x_20 >= x_min;
    x_21 >= x_min;
    x_22 >= x_min;
    x_23 >= x_min;
    x_24 >= x_min;
    x_25 >= x_min;
    x_25 == rf;
    x_0 == z_0 + Gamma_1h*u_0;
    x_1 == z_1 + Gamma_1h*u_1;
    x_2 == z_2 + Gamma_1h*u_2;
    x_3 == z_3 + Gamma_1h*u_3;
    x_4 == z_4 + Gamma_1h*u_4;
    x_5 == z_5 + Gamma_1h*u_5;
    x_6 == z_6 + Gamma_1h*u_6;
    x_7 == z_7 + Gamma_1h*u_7;
    x_8 == z_8 + Gamma_1h*u_8;
    x_9 == z_9 + Gamma_1h*u_9;
    x_10 == z_10 + Gamma_1h*u_10;
    x_11 == z_11 + Gamma_1h*u_11;
    x_12 == z_12 + Gamma_1h*u_12;
    x_13 == z_13 + Gamma_1h*u_13;
    x_14 == z_14 + Gamma_1h*u_14;
    x_15 == z_15 + Gamma_1h*u_15;
    x_16 == z_16 + Gamma_1h*u_16;
    x_17 == z_17 + Gamma_1h*u_17;
    x_18 == z_18 + Gamma_1h*u_18;
    x_19 == z_19 + Gamma_1h*u_19;
    x_20 == z_20 + Gamma_1h*u_20;
    x_21 == z_21 + Gamma_1h*u_21;
    x_22 == z_22 + Gamma_1h*u_22;
    x_23 == z_23 + Gamma_1h*u_23;
    x_24 == z_24 + Gamma_1h*u_24;
    x_25 == z_25;
cvx_end
vars.u_0 = u_0;
vars.u_1 = u_1;
vars.u{1} = u_1;
vars.u_2 = u_2;
vars.u{2} = u_2;
vars.u_3 = u_3;
vars.u{3} = u_3;
vars.u_4 = u_4;
vars.u{4} = u_4;
vars.u_5 = u_5;
vars.u{5} = u_5;
vars.u_6 = u_6;
vars.u{6} = u_6;
vars.u_7 = u_7;
vars.u{7} = u_7;
vars.u_8 = u_8;
vars.u{8} = u_8;
vars.u_9 = u_9;
vars.u{9} = u_9;
vars.u_10 = u_10;
vars.u{10} = u_10;
vars.u_11 = u_11;
vars.u{11} = u_11;
vars.u_12 = u_12;
vars.u{12} = u_12;
vars.u_13 = u_13;
vars.u{13} = u_13;
vars.u_14 = u_14;
vars.u{14} = u_14;
vars.u_15 = u_15;
vars.u{15} = u_15;
vars.u_16 = u_16;
vars.u{16} = u_16;
vars.u_17 = u_17;
vars.u{17} = u_17;
vars.u_18 = u_18;
vars.u{18} = u_18;
vars.u_19 = u_19;
vars.u{19} = u_19;
vars.u_20 = u_20;
vars.u{20} = u_20;
vars.u_21 = u_21;
vars.u{21} = u_21;
vars.u_22 = u_22;
vars.u{22} = u_22;
vars.u_23 = u_23;
vars.u{23} = u_23;
vars.u_24 = u_24;
vars.u{24} = u_24;
vars.x_1 = x_1;
vars.x{1} = x_1;
vars.x_2 = x_2;
vars.x{2} = x_2;
vars.x_3 = x_3;
vars.x{3} = x_3;
vars.x_4 = x_4;
vars.x{4} = x_4;
vars.x_5 = x_5;
vars.x{5} = x_5;
vars.x_6 = x_6;
vars.x{6} = x_6;
vars.x_7 = x_7;
vars.x{7} = x_7;
vars.x_8 = x_8;
vars.x{8} = x_8;
vars.x_9 = x_9;
vars.x{9} = x_9;
vars.x_10 = x_10;
vars.x{10} = x_10;
vars.x_11 = x_11;
vars.x{11} = x_11;
vars.x_12 = x_12;
vars.x{12} = x_12;
vars.x_13 = x_13;
vars.x{13} = x_13;
vars.x_14 = x_14;
vars.x{14} = x_14;
vars.x_15 = x_15;
vars.x{15} = x_15;
vars.x_16 = x_16;
vars.x{16} = x_16;
vars.x_17 = x_17;
vars.x{17} = x_17;
vars.x_18 = x_18;
vars.x{18} = x_18;
vars.x_19 = x_19;
vars.x{19} = x_19;
vars.x_20 = x_20;
vars.x{20} = x_20;
vars.x_21 = x_21;
vars.x{21} = x_21;
vars.x_22 = x_22;
vars.x{22} = x_22;
vars.x_23 = x_23;
vars.x{23} = x_23;
vars.x_24 = x_24;
vars.x{24} = x_24;
vars.x_25 = x_25;
vars.x{25} = x_25;
vars.z_0 = z_0;
vars.z_1 = z_1;
vars.z{1} = z_1;
vars.z_2 = z_2;
vars.z{2} = z_2;
vars.z_3 = z_3;
vars.z{3} = z_3;
vars.z_4 = z_4;
vars.z{4} = z_4;
vars.z_5 = z_5;
vars.z{5} = z_5;
vars.z_6 = z_6;
vars.z{6} = z_6;
vars.z_7 = z_7;
vars.z{7} = z_7;
vars.z_8 = z_8;
vars.z{8} = z_8;
vars.z_9 = z_9;
vars.z{9} = z_9;
vars.z_10 = z_10;
vars.z{10} = z_10;
vars.z_11 = z_11;
vars.z{11} = z_11;
vars.z_12 = z_12;
vars.z{12} = z_12;
vars.z_13 = z_13;
vars.z{13} = z_13;
vars.z_14 = z_14;
vars.z{14} = z_14;
vars.z_15 = z_15;
vars.z{15} = z_15;
vars.z_16 = z_16;
vars.z{16} = z_16;
vars.z_17 = z_17;
vars.z{17} = z_17;
vars.z_18 = z_18;
vars.z{18} = z_18;
vars.z_19 = z_19;
vars.z{19} = z_19;
vars.z_20 = z_20;
vars.z{20} = z_20;
vars.z_21 = z_21;
vars.z{21} = z_21;
vars.z_22 = z_22;
vars.z{22} = z_22;
vars.z_23 = z_23;
vars.z{23} = z_23;
vars.z_24 = z_24;
vars.z{24} = z_24;
vars.z_25 = z_25;
vars.z{25} = z_25;
status.cvx_status = cvx_status;
% Provide a drop-in replacement for csolve.
status.optval = cvx_optval;
status.converged = strcmp(cvx_status, 'Solved');
