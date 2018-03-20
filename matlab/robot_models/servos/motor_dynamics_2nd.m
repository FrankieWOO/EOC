function [ dx ] = motor_dynamics_2nd( theta, dtheta, theta_d )
%MOTOR_DYNAMICS_2ND 2nd order motor dynamics
%   
    beta = 30;
    accel = (beta^2)*(theta_d - theta) - 2*beta*dtheta;
    dx = [dtheta; accel];
end

