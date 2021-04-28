% csolve  Solves a custom quadratic program very rapidly.
%
% [vars, status] = csolve(params, settings)
%
% solves the convex optimization problem
%
%   minimize(quad_form(x_1, Q) + quad_form(x_2, Q) + quad_form(x_3, Q) + quad_form(x_4, Q) + quad_form(x_5, Q) + quad_form(x_6, Q) + quad_form(x_7, Q) + quad_form(x_8, Q) + quad_form(x_9, Q) + quad_form(x_10, Q) + quad_form(u_0, R) + quad_form(u_1, R) + quad_form(u_2, R) + quad_form(u_3, R) + quad_form(u_4, R) + quad_form(u_5, R) + quad_form(u_6, R) + quad_form(u_7, R) + quad_form(u_8, R) + quad_form(u_9, R))
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
%     x_10 == rf
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
%     x_10 == z_10
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
% Produced by CVXGEN, 2019-12-10 11:49:04 -0500.
% CVXGEN is Copyright (C) 2006-2017 Jacob Mattingley, jem@cvxgen.com.
% The code in this file is Copyright (C) 2006-2017 Jacob Mattingley.
% CVXGEN, or solvers produced by CVXGEN, cannot be used for commercial
% applications without prior written permission from Jacob Mattingley.

% Filename: csolve.m.
% Description: Help file for the Matlab solver interface.
