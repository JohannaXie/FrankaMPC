function [vars,status] = calculate_params(h,x_0,rf)
    params.A = [1 h h^2*0.5;0 1 h;0 0 1];
    params.B = [7*h^3/12;h^2;h];
    params.Gamma_1h = [h^3/24;h^2/6;h/2];
    params.Q = diag([0,1,1]);
    params.R = 0.001;
    params.x_max = [2.8973;2.6100;15.0];
    params.x_min = -params.x_max;
    params.u_max = 7500;
    params.x_0 = x_0;
    params.rf = rf;
    [vars, status] = csolve(params);
end