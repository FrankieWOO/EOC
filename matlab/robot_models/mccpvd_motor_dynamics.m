function [ dx ] = mccpvd_motor_dynamics( theta, dtheta, u )
%MCCPVD_MOTOR_DYNAMICS Summary of this function goes here
%   Detailed explanation goes 
    beta = 30;
    accel1 = (beta^2)*(u(1) - theta(1)) - 2*beta*dtheta(1);
    accel2 = (beta^2)*(u(2) - theta(2)) - 2*beta*dtheta(2);
    dx = [dtheta; accel1;accel2];
end

