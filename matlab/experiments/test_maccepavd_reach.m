%%%% step 1: define robot
% the maccepavd robot model has 8 state dimension

%param_act.ratio_load = 0;
param_act.ratio_load = 1;
param_act.gear_d = 100;
%param_act.Kd = 0.0212;
%param_act.K1 = 1;
%param_act.K2 = 1;
%param_act.R1 = 0.5;
%param_act.R2 = 0.5;
%param_act.Ks = 500;
%param_act.J1 = 0.001;
%param_act.J2 = 0.001;
robot_param.inertia_l = 0.0016;
%robot_param.Df = 0.01;
robot_model = Mccpvd1dofModel(robot_param, param_act);


%%%% step 2: define task
%---- user specification ----%
target = 0.7;
T = 1.5;
dt = 0.02;
N = T/dt + 1;
%alpha = 0.7;
position0 = 0;
x0 = zeros(6,1); 
x0(1) = position0;
x0(3) = position0;
x0(5) = 0;

% task_param = [];
% task_param.target = target;
% task_param.T = T;
% task_param.w = w;
% task_param.rege_ratio = alpha;
% task_param.position0 =  position0;
% task_param.x0 = x0; 
% %define the initial spring pretension motor state
% % all velocity, accel states are zero
% task_param.with_motor_dynamics = 1;
% task_param.multiple_inits = 0;
%----%

%task = OptTask(robot_model, 'fast_reach', 'net_mechpower', task_param);
%%%%

%%%% step 3: init OC handler

cost_param = [];
%cost_param.w0 = 1;

cost_param.w_e = 1e-2;
cost_param.w_t = 1;
cost_param.w_tf= 1*dt;
%cost_param.alpha = alpha;
cost_param.epsilon = 0;
cost_param.T = T;
cost_param.dt = dt;
cost_param.target = target;
cost_param.fd = 1; % use finite difference or not
cost_param.x0 = x0;

f = @(x,u)robot_model.dynamics_with_jacobian_fd(x,u);

task1 = mccpvd1_reach(robot_model, cost_param);
%cost_param2=cost_param;
%cost_param2.w_e = cost_param.w_e*(1e-3);
%task2 = mccpvd1_reach(robot_model, cost_param2);
j1 = @(x,u,t)task1.j_load_posi(x,u,t);
j2 = @(x,u,t)task1.j_load_posi_rege(x,u,t);

%j2 = @(x,u,t)task2.j_tf_elec(x,u,t);
%j3 = @(x,u,t)task2.j_tf_elec_rege(x,u,t);
%j = @(x,u,t)task.j_noutmech(x,u,t);
%costfn = CostMccvd1();
%j = @(x,u,t)costfn.j_reach_netmech(robot_model,x,u,t,cost_param);

%oc = ILQRController(robot_model, task);
%oc.opt_param.online_plotting=0;

%%
opt_param = [];
opt_param.umax = robot_model.umax;
opt_param.umin = robot_model.umin;
opt_param.lambda_init = 0.01;
opt_param.lambda_max  = 50000;
opt_param.iter_max = 250;
opt_param.online_plotting = 0;
opt_param.online_printing = 1;
opt_param.dcost_converge = 10^-8;
opt_param.solver = 'rk4';
opt_param.target = target;

opt_param.T = T;

% u0 can be full command sequence or just initial point
u0 = [cost_param.target; 0; 0];
%u0 = [0; 0.1; 0];

result1 = ILQRController.run_multiple(f, j1, dt, N, x0, u0, opt_param);
result2 = ILQRController.run_multiple(f, j2, dt, N, x0, u0, opt_param);
%result3 = ILQRController.ilqr(f, j3, dt, N, x0, result1.u, opt_param);

%result = ILQRController.run_multiple(f, j, dt, N, x0, u0, opt_param);
%%
t = 0:dt:T;
tsim = 0:0.001:T;
usim1 = scale_controlSeq(result1.u,t,tsim);
psim.solver = 'rk4';
psim.dt = 0.001;
[xsim1] = simulate_feedforward(x0,f,usim1,psim);

result2.usim = scale_controlSeq(result2.u,t,tsim);
result2.xsim = simulate_feedforward(x0,f,result2.usim,psim);
%result3.usim = scale_controlSeq(result3.u,t,tsim);
%result3.xsim = simulate_feedforward(x0,f,result3.usim,psim);

tjf1 = traj_features(robot_model,result1.x,result1.u,0.02, cost_param);
tjf2 = traj_features(robot_model,result2.x,result2.u,0.02, cost_param);
%tjf3 = traj_features(robot_model,result3.x,result3.u,0.02);
tjf1sim = traj_features(robot_model, xsim1,usim1,0.001, cost_param);
tjf2sim = traj_features(robot_model, result2.xsim,result2.usim,0.001, cost_param);
%tjf3sim = traj_features(robot_model, result3.xsim,result3.usim,0.001);

figure
plot(result1.x(1,:))
hold on
plot(result2.x(1,:))
%plot(result3.x(1,:))
hold off

figure
plot(result1.u(1,:))
hold on
plot(result2.u(1,:))
%plot(result3.u(1,:))
hold off

figure
plot(result1.u(2,:))
hold on
plot(result2.u(2,:))
%plot(result3.u(2,:))
hold off

figure
plot(result1.u(3,:))
hold on
plot(result2.u(3,:))
%plot(result3.u(3,:))
hold off