%function simmccpa_example_iLQG
%param_act.ratio_load = 0;

clc
clear all

param_act.ratio_load = 1;
param_act.gear_d = 40;
%param_act.Kd = 0.0212;
%param_act.K1 = 1;
%param_act.K2 = 1;
%param_act.R1 = 0.5;
%param_act.R2 = 0.5;
%param_act.Ks = 500;
%param_act.J1 = 0.001;
%param_act.J2 = 0.001;
robot_param=[];
robot_param.inertia_l = 0.0015;
robot_param.Df = 0.004;
robot_model = Mccpvd1dofModel(robot_param, param_act);

target = pi/4;
T = 2;
dt = 0.02; %control time step
N = T/dt + 1;
t = 0:dt:T;
%alpha = 0.7;
position0 = 0;
x0 = zeros(6,1); 
x0(1) = 0;
x0(3) = 0; % initial motor1
x0(4) = pi/6; % initial motor2

%%%% step 3: init OC handler

cost_param = [];
%cost_param.w0 = 1;

cost_param.w_e = 1e0;
cost_param.w_t = 1e0;
cost_param.w_tf = 1e3;
cost_param.w_r = 0;
%cost_param.alpha = alpha;
cost_param.epsilon = 1e-6;
cost_param.T = T;
cost_param.dt = dt;
cost_param.target = target;
cost_param.fd = 1; % use finite difference or not
cost_param.x0 = x0;

% robot dynamics
f = @(x,u)robot_model.dynamics(x,u);
%disdyn = @(x,u)discrete_dynamics(f,x,u);
task = mccpvd1_reach(robot_model, cost_param);

% cost functions
%cost_param2=cost_param;
%cost_param2.w_e = cost_param.w_e*(1e-3);
%task2 = mccpvd1_reach(robot_model, cost_param2);
%j1 = @(x,u,t)task1.j_spf(x,u,t);
j2 = @(x,u,t)task.j_spf_rege(x,u,t);
%disj1 = @(x, u, t) discrete_cost(j1, x, u, t);
%disj2 = @(x, u, t) discrete_cost(j2, x, u, t);

%dyncst1 = @(x, u, i)dyna_cost(f, j1, x, u, i);
dyncst2 = @(x, u, i)dyna_cost(f, j2, x, u, i);
%%
Op = [];
Op.lambda = 0.01;
%Op.dlambda = 0.01;
%Op.lambdaMax = 1e20;
%Op.lambdaFactor = 1.2;
%full_DDP = false;


Op.maxIter = 200;
Op.tolGrad = 1e-4;
%Op.print = 1;

u0 = [target; pi/6; 0];
%u0 = [0.5;0.1;0.2];
u0 = repmat(u0, 1, N-1);

Op.lims = [robot_model.umin, robot_model.umax];
%result3 = ILQRController.ilqr(f, j2, dt, N, x0, u0, opt_param3);
result1 = TrajOptimiserILQG.iLQG_wrapper(dyncst2, x0, u0, Op);

%% evaluate the result
for i = 1:N-1
    result1.k(i) = robot_model.stiffness( result1.x(:,i));
    result1.d(i) = robot_model.damping( result1.u(:,i));
    
end

traj = val_traj_mccpvd(robot_model, task, result1);
%%
traj.t =t;
plot_traj_mccpvd1(traj);