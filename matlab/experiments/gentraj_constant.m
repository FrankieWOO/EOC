function [ traj ] = gentraj_constant( x0, target, u2, d )
%GENTRAJ_CONSTANT Summary of this function goes here
%   Detailed explanation goes here
T = 1.5;
dt = 0.02;
N = T/dt;
u = repmat([target;u2;d],1,N);
traj.u = u;
traj.t = 0:dt:T;

%param_act.ratio_load = 1;
param_act.gear_d = 40;
%param_act.Kd = 0.0212;
%param_act.K1 = 1;
%param_act.K2 = 1;
%param_act.R1 = 0.5;
%param_act.R2 = 0.5;
%param_act.Ks = 500;
%param_act.J1 = 0.001;
%param_act.J2 = 0.001;
robot_param.inertia = 3.6e-3;
robot_param.Df = 7.7e-3;
robot_model = Mccpvd1dofModel(robot_param, param_act);

cost_param = [];
%cost_param.w0 = 1;

cost_param.w_e = 1e0;
cost_param.w_t = 1e3;
cost_param.w_tf = cost_param.w_t*dt;
cost_param.w_r = 0;
%cost_param.alpha = alpha;
cost_param.epsilon = 1e-6;
cost_param.T = T;
cost_param.dt = dt;
cost_param.target = target;
cost_param.fd = 1; % use finite difference or not
cost_param.x0 = x0;

task = mccpvd1_reach(robot_model, cost_param);
traj.x = x0;
traj = val_traj_mccpvd(robot_model, task, traj);

end

