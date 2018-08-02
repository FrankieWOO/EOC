function [ dx ] = motor_dynamics_2nd( theta, dtheta, theta_d, beta )
%MOTOR_DYNAMICS_2ND 2nd order motor dynamics
%   
    if nargin == 3
    beta = 25;
    end
    accel = (beta^2)*(theta_d - theta) - 2*beta*dtheta;
    dx = [dtheta; accel];
end

