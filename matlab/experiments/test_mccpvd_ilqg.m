%function simmccpa_example_iLQG
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
robot_param=[];
robot_param.inertia_l = 0.0015;
robot_param.Df = 0.004;
robot_model = Mccpvd1dofModel(robot_param, param_act);

target = pi/4;
T = 2;
dt = 0.02;
N = T/dt + 1;
t = 0:dt:T;
%alpha = 0.7;
position0 = 0;
x0 = zeros(6,1); 
x0(1) = 0;
x0(3) = 0; % initial motor1
x0(4) = 0; % initial motor2

%%%% step 3: init OC handler

cost_param = [];
%cost_param.w0 = 1;

cost_param.w_e = 1e0;
cost_param.w_t = 1e3;
cost_param.w_tf = cost_param.w_t;
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
task1 = mccpvd1_reach(robot_model, cost_param);

% cost functions
%cost_param2=cost_param;
%cost_param2.w_e = cost_param.w_e*(1e-3);
%task2 = mccpvd1_reach(robot_model, cost_param2);
%j1 = @(x,u,t)task1.j_spf(x,u,t);
j2 = @(x,u,t)task1.j_spf_rege(x,u,t);
%disj1 = @(x, u, t) discrete_cost(j1, x, u, t);
%disj2 = @(x, u, t) discrete_cost(j2, x, u, t);

dyncst1 = @(x, u, i) dyna_cost(f, j1, x, u, i);
dyncst2 = @(x, u, i)dyna_cost(f, j2, x, u, i);
% control limits
Op.lims = [robot_model.umin, robot_model.umax];
%Op.lambda = 0.01;
%Op.dlambda = 0.01;
%Op.lambdaMax = 1e20;
%Op.lambdaFactor = 1.2;
%full_DDP = false;
opt_param = [];
opt_param.umax = robot_model.umax; 
opt_param.umin = robot_model.umin;
opt_param.lambda_init = 1;
opt_param.lambda_max  = 1e10;
opt_param.iter_max = 300;
opt_param.online_plotting = 0;
opt_param.online_printing = 1;
opt_param.dcost_converge = 10^-4;
opt_param.solver = 'rk4';
opt_param.target = target;

opt_param.T = T;

%opt_param.umin(1) = target;
%opt_param.umax(1) = target;

%opt_param.umin(2) = x0(4);
%opt_param.umax(2) = x0(4);

% u0 can be full command sequence or just initial point
%u0 = result0.u;
u0 = [target; 0; 0];
%u0 = [0.5;0.1;0.2];
u0 = repmat(u0, 1, N-1);


%% critical damped
% psim.dt = 0.02;
% psim.solver = 'rk4';
% 
% result0.u = repmat([target; 0.1; 0],1,N-1);
% result0.x = repmat(x0,1,N);
% for i = 1:N-1
%     result0.k(i) = robot_model.stiffness( result0.x(:,i) );
%     result0.d(i) = min(robot_model.actuator.max_damping,max(0,2*sqrt(result0.k(i)*robot_model.inertia) - robot_model.Df));
%     result0.u(3,i) = result0.d(i)/robot_model.actuator.max_damping;
%     result0.x(:,i+1) = simulate_step(f,result0.x(:,i),result0.u(:,i),psim);
%     
% end


%% pure regenerative braking
% u3_maxrege = robot_model.actuator.u_max_regedamp;
% opt_param2 = opt_param;
% opt_param2.umax(3) = u3_maxrege;
% Op.lims = [opt_param2.umin, opt_param2.umax];
% %result2 = ILQRController.ilqr(f, j1, dt, N, x0, u0, opt_param2);
% result2 = iLQG_wrapper(dyncst1, x0, u0, Op);
% for i = 1:N-1
%     result2.k(i) = robot_model.stiffness( result2.x(:,i));
%     result2.d(i) = robot_model.damping( result2.u(:,i));
%     
% end
%% %% hybrid mode
opt_param1 = opt_param;
opt_param1.umax(3) = 1;
Op.lims = [opt_param1.umin, opt_param1.umax];
%result3 = ILQRController.ilqr(f, j2, dt, N, x0, u0, opt_param3);
result1 = TrajOptimiserILQG.iLQG_wrapper(dyncst2, x0, u0, Op);

%% evaluate the result
for i = 1:N-1
    result1.k(i) = robot_model.stiffness( result1.x(:,i));
    result1.d(i) = robot_model.damping( result1.u(:,i));
    
end

% simulate the traj using very small dt
tsim = 0:0.001:T;
psim.solver = 'rk4';
psim.dt = 0.001;

result1.usim = scale_controlSeq(result1.u,t(1:end-1),tsim(1:end-1));

result1.xsim = simulate_feedforward(x0,f,result1.usim,psim);

for i=1:length(tsim)-1
    
    result1.Prege(i) = robot_model.power_rege( result1.xsim(:,i),result1.usim(:,i));
    
end

% Energy
result1.Erege = sum(result1.Prege)*psim.dt;

for i = 1:length(tsim)-1
    
    result1.Plink(i) = robot_model.power_link(result1.xsim(:,i), result1.usim(:,i));
    
end

result1.E = sum(result1.Plink)*psim.dt;
result1.T = T;
result1.t = t;

