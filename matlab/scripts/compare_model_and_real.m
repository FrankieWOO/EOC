%param_act.ratio_load = 0;
param_act.ratio_load = 1;
param_act.gear_d = 20;
%param_act.Kd = 0.0212;
%param_act.K1 = 1;
%param_act.K2 = 1;
%param_act.R1 = 0.5;
%param_act.R2 = 0.5;
%param_act.Ks = 500;
%param_act.J1 = 0.001;
%param_act.J2 = 0.001;
robot_param=[];
%robot_param.inertia_l = 0.0016;
robot_param.Df = 0.005; %seems 0.0023 is too small for swing from 0.
robot_model = Mccpvd1dofModel(robot_param, param_act);
dynfn = @(x,u)robot_model.dynamics(x,u);


%% Define commands
N_trajs = 3;
N_t = 100;
U = cell(N_trajs,1);

zero_traj = repmat([0;0.5;0],1,N_t);

zero_traj_h = repmat([0;0.5;1],1,N_t);
lowdamp = repmat( [0.8;0.5;0],1, N_t);
middamp = repmat( [0.8;0.5;0.5],1, N_t);
highdamp= repmat([0.8;0.5;1],1,N_t);
U{1} = lowdamp;
U{2} = middamp;
U{3} = highdamp;
%U{4} = zero_traj;

zero_middamp = repmat( [0;0.8;0.5],1, N_t);
reset = repmat( [0;0;0],1, N_t);
%%
u = middamp;
y = exetraj(u);
y_rp = (y(2,:)/1000).^2 * 25;
%% run prediction
T = 2;
dt = 0.02;
N = T/dt + 1;
t = 0:dt:T;
%alpha = 0.7;
position0 = 0;
x0 = zeros(6,1); 
x0(1) = position0;
x0(3) = 0; % initial motor1
x0(4) = 0; % initial motor2

ypred = predict_traj(robot_model, x0, u, t);
%%
figure
subplot(2,1,1)
hold on
plot(t,ypred(1,:))
plot(t,y(1,:))
legend('prediction','recorded')
title('position')
hold off

subplot(2,1,2)
hold on
plot(t, ypred(7,:))
plot(t, y_rp)
legend('prediction','recorded')
title('rege power')
hold off