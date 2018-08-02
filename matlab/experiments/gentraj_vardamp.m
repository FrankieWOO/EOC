function [ traj ] = gentraj_vardamp( x0, target, u2 )

%%%% step 1: define robot
% the maccepavd robot model has 8 state dimension
min_preload = pi/6;

if nargin > 0
    x0 = x0;
    target = target;
    u2 = u2;
else
    q0 = 0;
    target = pi/4;
    u2 = pi/6;
    
    x0 = [q0;0; q0; min_preload;0;0  ];
end


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
%robot_param.inertia_l = 0.0015;
robot_param.inertia = 3.6e-3;
robot_param.Df = 7.7e-3;
robot_model = Mccpvd1dofModel(robot_param, param_act);


%%%% step 2: define task
%---- user specification ----%
T = 1.5;
dt = 0.02;
t = 0:dt:T;
N = T/dt + 1;
%alpha = 0.7;



%%%% step 3: set cost function parameters

cost_param = [];
%cost_param.w0 = 1;

cost_param.w_e = 1e0;
cost_param.w_t = 1e3;
cost_param.w_tf = cost_param.w_t*dt;
cost_param.w_r = 5e2;
%cost_param.alpha = alpha;
cost_param.epsilon = 1e-6;
cost_param.T = T;
cost_param.dt = dt;
cost_param.target = target;
cost_param.fd = 1; % use finite difference or not
cost_param.x0 = x0;
%% init 
f = @(x,u)robot_model.dynamics_with_jacobian_fd(x,u);

task = mccpvd1_reach(robot_model, cost_param);
%cost_param2=cost_param;
%cost_param2.w_e = cost_param.w_e*(1e-3);
%task2 = mccpvd1_reach(robot_model, cost_param2);
%j1 = @(x,u,t)task1.j_effort(x,u,t);


% --- 
% c = c + self.w_r* (u(3,:)-0.5).^2;
% ---
j = @(x,u,t)task.j_spf_rege(x,u,t);

%j2 = @(x,u,t)task2.j_tf_elec(x,u,t);
%j3 = @(x,u,t)task2.j_tf_elec_rege(x,u,t);
%j = @(x,u,t)task.j_noutmech(x,u,t);
%costfn = CostMccvd1();
%j = @(x,u,t)costfn.j_reach_netmech(robot_model,x,u,t,cost_param);

%oc = ILQRController(robot_model, task);
%oc.opt_param.online_plotting=0;

%% optimisation
opt_param = [];
opt_param.umax = robot_model.umax; 
opt_param.umin = robot_model.umin;
opt_param.lambda_init = 1;
opt_param.lambda_max  = 1e10;
opt_param.iter_max = 150;
opt_param.online_plotting = 0;
opt_param.online_printing = 1;
opt_param.dcost_converge = 1e-4;
opt_param.solver = 'rk4';
opt_param.target = target;

opt_param.T = T;

opt_param.umax = [target; u2; 1];
opt_param.umin = [target; u2; 0];
% u0 can be full command sequence or just initial point
u0 = [cost_param.target; u2; 0];
%u0 = [0; 0.1; 0];

traj = ILQRController.ilqr(f, j, dt, N, x0, u0, opt_param);
%result2 = ILQRController.ilqr(f, j2, dt, N, x0, u0, opt_param);
%result3 = ILQRController.ilqr(f, j3, dt, N, x0, result.u, opt_param);

%result = ILQRController.run_multiple(f, j, dt, N, x0, u0, opt_param);
%% Do a forward pass to evaluate the trajectory with dt = 0.001 for simulation


%result2.usim = scale_controlSeq(result2.u,t,tsim);
%result2.xsim = simulate_feedforward(x0,f,result2.usim,psim);
%result3.usim = scale_controlSeq(result3.u,t,tsim);
%result3.xsim = simulate_feedforward(x0,f,result3.usim,psim);

%tjf1 = traj_features(robot_model,result.x,result.u,0.02, cost_param);
%tjf2 = traj_features(robot_model,result2.x,result2.u,0.02, cost_param);
%tjf3 = traj_features(robot_model,result3.x,result3.u,0.02);
%tjf1sim = traj_features(robot_model, xsim1,usim1,0.001, cost_param);
%tjf2sim = traj_features(robot_model, result2.xsim,result2.usim,0.001, cost_param);
%tjf3sim = traj_features(robot_model, result3.xsim,result3.usim,0.001);
traj.t =t;
traj = val_traj_mccpvd(robot_model, task, traj);

if nargin == 0
    plot_traj_mccpvd1(traj);   
end

%

%assignin('base', 'traj', traj);

%save('tjlib/vardamp/test.mat','traj');

%%
% 
% uu3 = 0:0.01:1;
% for i=1:length(uu3)
%     dd(i) = robot.actuator.damping(uu3(i));
%     pp(i) = robot.actuator.power_rege(1,uu3(i));
% end
% figure
% plot(uu3, dd, uu3, pp)

end

