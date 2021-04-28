% Produced by CVXGEN, 2019-12-10 11:49:04 -0500.
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

  minimize(quad_form(x_1, Q) + quad_form(x_2, Q) + quad_form(x_3, Q) + quad_form(x_4, Q) + quad_form(x_5, Q) + quad_form(x_6, Q) + quad_form(x_7, Q) + quad_form(x_8, Q) + quad_form(x_9, Q) + quad_form(x_10, Q) + quad_form(u_0, R) + quad_form(u_1, R) + quad_form(u_2, R) + quad_form(u_3, R) + quad_form(u_4, R) + quad_form(u_5, R) + quad_form(u_6, R) + quad_form(u_7, R) + quad_form(u_8, R) + quad_form(u_9, R));
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
    x_10 == rf;
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
    x_10 == z_10;
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
status.cvx_status = cvx_status;
% Provide a drop-in replacement for csolve.
status.optval = cvx_optval;
status.converged = strcmp(cvx_status, 'Solved');
