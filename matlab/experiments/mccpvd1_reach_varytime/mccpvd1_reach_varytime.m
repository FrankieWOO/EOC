% reaching by varying terminal time

rng(1)
% the maccepavd robot model has 8 state dimension

%param_act.ratio_load = 0;
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
robot_param.inertia_l = 0.0016;
%robot_param.Df = 0.01;
robot_model = Mccpvd1dofModel(robot_param);
robot_model.actuator = ActMccpvd(param_act);

%%%% step 2: define task
%---- user specification ----%
Ts = 0.5:0.1:1.5;
target = 0.7;
dt = 0.02;

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

cost_param.w_e = 1e-4;
cost_param.w_t = 0;
cost_param.w_tf= 1;
%cost_param.alpha = alpha;
cost_param.epsilon = 0;

cost_param.dt = dt;
cost_param.target = target;
cost_param.fd = 1; % use finite difference or not
cost_param.x0 = x0;

opt_param = [];
opt_param.umax = robot_model.umax;
opt_param.umin = robot_model.umin;
opt_param.lambda_init = 0.01;
opt_param.lambda_max  = 0.5;
opt_param.iter_max = 100;
opt_param.online_plotting = 0;
opt_param.online_printing = 1;
opt_param.dcost_converge = 10^-8;
opt_param.solver = 'rk4';
opt_param.target = target;

f = @(x,u)robot_model.dynamics_with_jacobian_fd(x,u);

tasks = cell(length(Ts),1);
results_elec = cell(size(tasks));
results_elec_rege = cell(size(tasks));

for i =1:length(tasks)
    cost_param.T = Ts(i);
    N = Ts(i)/dt + 1;
    tasks{i} = mccpvd1_reach(robot_model, cost_param);
    
    
    opt_param.T = Ts(i);
    
    % u0 can be full command sequence or just initial point
    u0 = [cost_param.target; 0; 0];
    %u0 = [0; 0.1; 0];
    j1 = @(x,u,t)tasks{i}.j_tf_elec(x,u,t);
    j2 = @(x,u,t)tasks{i}.j_tf_elec_rege(x,u,t);
    traj_elec = ILQRController.run_multiple(f, j1, dt, N, x0, u0, opt_param);
    traj_elec_rege = ILQRController.run_multiple(f, j2, dt, N, x0, u0, opt_param);
    results_elec{i}.traj = traj_elec;
    results_elec_rege{i}.traj = traj_elec_rege;
    t = 0:dt:Ts(i);
    tsim = 0:0.001:Ts(i) ;
    psim.solver = 'rk4';
    psim.dt = 0.001;
    traj_elec.usim = scale_controlSeq(traj_elec.u,t,tsim);
    
    [traj_elec.xsim] = simulate_feedforward(x0,f,traj_elec.usim,psim);
    traj_elec_rege.usim = scale_controlSeq(traj_elec_rege.u,t,tsim);
    
    [traj_elec_rege.xsim] = simulate_feedforward(x0,f,traj_elec_rege.usim,psim);
    
    results_elec{i}.tjf = traj_features(robot_model, results_elec{i}.traj.xsim, results_elec{i}.traj.usim, 0.001);
    results_elec_rege{i}.tjf = traj_features(robot_model, results_elec_rege{i}.traj.xsim, results_elec_rege{i}.traj.usim, 0.001);
    
end

