function [ traj ] = gentraj_constant( target, u2, d )
%GENTRAJ_CONSTANT Summary of this function goes here
%   Detailed explanation goes here
T = 2;
dt = 0.02;
N = T/dt;
u = repmat([target;u2;d],1,N);
traj.u = u;
traj.t = 0:dt:T;

end

