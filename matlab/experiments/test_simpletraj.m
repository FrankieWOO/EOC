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
robot_param.inertia_l = 0.01;
robot_param.Df = 0.01;
robot_model = Mccpvd1dofModel(robot_param);
robot_model.actuator = ActMccpvd(param_act);
f = @(x,u)robot_model.dynamics_with_jacobian_fd(x,u);
%%%% step 2: define task
%---- user specification ----%
target = 0.7;
T = 0.3;
dt = 0.02;
N = T/dt + 1;
w = 0.1;
w_e = 0.001;
%alpha = 0.7;
position0 = 0;
x0 = zeros(6,1); 
x0(1) = position0;
x0(3) = position0;
x0(5) = 0;


%%
% test simple traj

p.dt = 0.02;
p.solver = 'rk4';
Nu = 25;

u = repmat([target;1;1],1,Nu);
t = 0:p.dt:2;
x = simulate_feedforward(x0,f,u,p);
tjf = traj_features(robot_model, x,u,0.02);
figure
plot(x(1,:))