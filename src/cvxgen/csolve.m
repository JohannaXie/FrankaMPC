% csolve  Solves a custom quadratic program very rapidly.
%
% [vars, status] = csolve(params, settings)
%
% solves the convex optimization problem
%
%   minimize(quad_form(x_1, Q) + quad_form(x_2, Q) + quad_form(x_3, Q) + quad_form(x_4, Q) + quad_form(x_5, Q) + quad_form(x_6, Q) + quad_form(x_7, Q) + quad_form(x_8, Q) + quad_form(x_9, Q) + quad_form(x_10, Q) + quad_form(x_11, Q) + quad_form(x_12, Q) + quad_form(x_13, Q) + quad_form(x_14, Q) + quad_form(x_15, Q) + quad_form(x_16, Q) + quad_form(x_17, Q) + quad_form(x_18, Q) + quad_form(x_19, Q) + quad_form(x_20, Q) + quad_form(x_21, Q) + quad_form(x_22, Q) + quad_form(x_23, Q) + quad_form(x_24, Q) + quad_form(x_25, Q) + quad_form(u_0, R) + quad_form(u_1, R) + quad_form(u_2, R) + quad_form(u_3, R) + quad_form(u_4, R) + quad_form(u_5, R) + quad_form(u_6, R) + quad_form(u_7, R) + quad_form(u_8, R) + quad_form(u_9, R) + quad_form(u_10, R) + quad_form(u_11, R) + quad_form(u_12, R) + quad_form(u_13, R) + quad_form(u_14, R) + quad_form(u_15, R) + quad_form(u_16, R) + quad_form(u_17, R) + quad_form(u_18, R) + quad_form(u_19, R) + quad_form(u_20, R) + quad_form(u_21, R) + quad_form(u_22, R) + quad_form(u_23, R) + quad_form(u_24, R))
%   subject to
%     z_1 == A*z_0 + B*u_0
%     z_2 == A*z_1 + B*u_1
%     z_3 == A*z_2 + B*u_2
%     z_4 == A*z_3 + B*u_3
%     z_5 == A*z_4 + B*u_4
%     z_6 == A*z_5 + B*u_5
%     z_7 == A*z_6 + B*u_6
%     z_8 == A*z_7 + B*u_7
%     z_9 == A*z_8 + B*u_8
%     z_10 == A*z_9 + B*u_9
%     z_11 == A*z_10 + B*u_10
%     z_12 == A*z_11 + B*u_11
%     z_13 == A*z_12 + B*u_12
%     z_14 == A*z_13 + B*u_13
%     z_15 == A*z_14 + B*u_14
%     z_16 == A*z_15 + B*u_15
%     z_17 == A*z_16 + B*u_16
%     z_18 == A*z_17 + B*u_17
%     z_19 == A*z_18 + B*u_18
%     z_20 == A*z_19 + B*u_19
%     z_21 == A*z_20 + B*u_20
%     z_22 == A*z_21 + B*u_21
%     z_23 == A*z_22 + B*u_22
%     z_24 == A*z_23 + B*u_23
%     z_25 == A*z_24 + B*u_24
%     abs(u_0) <= u_max
%     abs(u_1) <= u_max
%     abs(u_2) <= u_max
%     abs(u_3) <= u_max
%     abs(u_4) <= u_max
%     abs(u_5) <= u_max
%     abs(u_6) <= u_max
%     abs(u_7) <= u_max
%     abs(u_8) <= u_max
%     abs(u_9) <= u_max
%     abs(u_10) <= u_max
%     abs(u_11) <= u_max
%     abs(u_12) <= u_max
%     abs(u_13) <= u_max
%     abs(u_14) <= u_max
%     abs(u_15) <= u_max
%     abs(u_16) <= u_max
%     abs(u_17) <= u_max
%     abs(u_18) <= u_max
%     abs(u_19) <= u_max
%     abs(u_20) <= u_max
%     abs(u_21) <= u_max
%     abs(u_22) <= u_max
%     abs(u_23) <= u_max
%     abs(u_24) <= u_max
%     x_1 <= x_max
%     x_2 <= x_max
%     x_3 <= x_max
%     x_4 <= x_max
%     x_5 <= x_max
%     x_6 <= x_max
%     x_7 <= x_max
%     x_8 <= x_max
%     x_9 <= x_max
%     x_10 <= x_max
%     x_11 <= x_max
%     x_12 <= x_max
%     x_13 <= x_max
%     x_14 <= x_max
%     x_15 <= x_max
%     x_16 <= x_max
%     x_17 <= x_max
%     x_18 <= x_max
%     x_19 <= x_max
%     x_20 <= x_max
%     x_21 <= x_max
%     x_22 <= x_max
%     x_23 <= x_max
%     x_24 <= x_max
%     x_25 <= x_max
%     x_1 >= x_min
%     x_2 >= x_min
%     x_3 >= x_min
%     x_4 >= x_min
%     x_5 >= x_min
%     x_6 >= x_min
%     x_7 >= x_min
%     x_8 >= x_min
%     x_9 >= x_min
%     x_10 >= x_min
%     x_11 >= x_min
%     x_12 >= x_min
%     x_13 >= x_min
%     x_14 >= x_min
%     x_15 >= x_min
%     x_16 >= x_min
%     x_17 >= x_min
%     x_18 >= x_min
%     x_19 >= x_min
%     x_20 >= x_min
%     x_21 >= x_min
%     x_22 >= x_min
%     x_23 >= x_min
%     x_24 >= x_min
%     x_25 >= x_min
%     x_25 == rf
%     x_0 == z_0 + Gamma_1h*u_0
%     x_1 == z_1 + Gamma_1h*u_1
%     x_2 == z_2 + Gamma_1h*u_2
%     x_3 == z_3 + Gamma_1h*u_3
%     x_4 == z_4 + Gamma_1h*u_4
%     x_5 == z_5 + Gamma_1h*u_5
%     x_6 == z_6 + Gamma_1h*u_6
%     x_7 == z_7 + Gamma_1h*u_7
%     x_8 == z_8 + Gamma_1h*u_8
%     x_9 == z_9 + Gamma_1h*u_9
%     x_10 == z_10 + Gamma_1h*u_10
%     x_11 == z_11 + Gamma_1h*u_11
%     x_12 == z_12 + Gamma_1h*u_12
%     x_13 == z_13 + Gamma_1h*u_13
%     x_14 == z_14 + Gamma_1h*u_14
%     x_15 == z_15 + Gamma_1h*u_15
%     x_16 == z_16 + Gamma_1h*u_16
%     x_17 == z_17 + Gamma_1h*u_17
%     x_18 == z_18 + Gamma_1h*u_18
%     x_19 == z_19 + Gamma_1h*u_19
%     x_20 == z_20 + Gamma_1h*u_20
%     x_21 == z_21 + Gamma_1h*u_21
%     x_22 == z_22 + Gamma_1h*u_22
%     x_23 == z_23 + Gamma_1h*u_23
%     x_24 == z_24 + Gamma_1h*u_24
%     x_25 == z_25
%
% with variables
%      u_0   1 x 1
%      u_1   1 x 1
%      u_2   1 x 1
%      u_3   1 x 1
%      u_4   1 x 1
%      u_5   1 x 1
%      u_6   1 x 1
%      u_7   1 x 1
%      u_8   1 x 1
%      u_9   1 x 1
%     u_10   1 x 1
%     u_11   1 x 1
%     u_12   1 x 1
%     u_13   1 x 1
%     u_14   1 x 1
%     u_15   1 x 1
%     u_16   1 x 1
%     u_17   1 x 1
%     u_18   1 x 1
%     u_19   1 x 1
%     u_20   1 x 1
%     u_21   1 x 1
%     u_22   1 x 1
%     u_23   1 x 1
%     u_24   1 x 1
%      x_1   3 x 1
%      x_2   3 x 1
%      x_3   3 x 1
%      x_4   3 x 1
%      x_5   3 x 1
%      x_6   3 x 1
%      x_7   3 x 1
%      x_8   3 x 1
%      x_9   3 x 1
%     x_10   3 x 1
%     x_11   3 x 1
%     x_12   3 x 1
%     x_13   3 x 1
%     x_14   3 x 1
%     x_15   3 x 1
%     x_16   3 x 1
%     x_17   3 x 1
%     x_18   3 x 1
%     x_19   3 x 1
%     x_20   3 x 1
%     x_21   3 x 1
%     x_22   3 x 1
%     x_23   3 x 1
%     x_24   3 x 1
%     x_25   3 x 1
%      z_0   3 x 1
%      z_1   3 x 1
%      z_2   3 x 1
%      z_3   3 x 1
%      z_4   3 x 1
%      z_5   3 x 1
%      z_6   3 x 1
%      z_7   3 x 1
%      z_8   3 x 1
%      z_9   3 x 1
%     z_10   3 x 1
%     z_11   3 x 1
%     z_12   3 x 1
%     z_13   3 x 1
%     z_14   3 x 1
%     z_15   3 x 1
%     z_16   3 x 1
%     z_17   3 x 1
%     z_18   3 x 1
%     z_19   3 x 1
%     z_20   3 x 1
%     z_21   3 x 1
%     z_22   3 x 1
%     z_23   3 x 1
%     z_24   3 x 1
%     z_25   3 x 1
%
% and parameters
%        A   3 x 3
%        B   3 x 1
% Gamma_1h   3 x 1
%        Q   3 x 3    PSD
%        R   1 x 1    PSD
%       rf   3 x 1
%    u_max   1 x 1    positive
%      x_0   3 x 1
%    x_max   3 x 1    positive
%    x_min   3 x 1    negative
%
% Note:
%   - Check status.converged, which will be 1 if optimization succeeded.
%   - You don't have to specify settings if you don't want to.
%   - To hide output, use settings.verbose = 0.
%   - To change iterations, use settings.max_iters = 20.
%   - You may wish to compare with cvxsolve to check the solver is correct.
%
% Specify params.A, ..., params.x_min, then run
%   [vars, status] = csolve(params, settings)
% Produced by CVXGEN, 2019-12-21 11:21:06 -0500.
% CVXGEN is Copyright (C) 2006-2017 Jacob Mattingley, jem@cvxgen.com.
% The code in this file is Copyright (C) 2006-2017 Jacob Mattingley.
% CVXGEN, or solvers produced by CVXGEN, cannot be used for commercial
% applications without prior written permission from Jacob Mattingley.

% Filename: csolve.m.
% Description: Help file for the Matlab solver interface.
