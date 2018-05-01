%%%% specify the robot model
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
%robot_param.inertia = 0.00315;
%robot_param.Df = 0.0062; %seems 0.0023 is too small for swing from 0.
robot_param.inertia = 0.0006; %0.0015;
robot_param.Df = 0.0104; %0.004; 

robot_model = Mccpvd1dofModel(robot_param, param_act);
%dynfn = @(x,u)robot_model.dynamics(x,u);

%%%%
%% Define commands
T = 2;
dt = 0.02;
N_t = T/dt + 1;
N_u = N_t-1;
t = 0:dt:T;
%%%% specify the initial state and target
position0 = 0;
motor2 = pi/3;
qtarget = pi/4;

x0 = zeros(6,1); 
x0(1) = position0;
x0(3) = position0; % initial motor1
x0(4) = motor2; % initial motor2
%%%%
reset0 = repmat( [0;0;0],1, N_u);
reset_traj_h = repmat([position0; pi/2; 0], 1, N_u);
reset_traj = repmat([position0; motor2; 0], 1, N_u);
reset_traj = [reset_traj_h, reset_traj];

N_trajs = 3;
U = cell(N_trajs,1);

lowdamp = repmat( [qtarget; motor2; 0],1, N_u);
middamp = repmat( [qtarget; motor2; 0.5],1, N_u);
highdamp= repmat([qtarget; motor2; 1],1,N_u);
U{1} = lowdamp;
U{2} = middamp;
U{3} = highdamp;
%U{4} = zero_traj;

 %% execute trajs
Y = cell(N_trajs,1);
for i=1:N_trajs

u = U{i};
y = exetraj(u);
y_rp = (y(2,:)/1000).^2 * 25;
y(2,:) = y_rp;
Y{i} = y;
    
    disp('Execution of traj done. Press keys to reset...')
    pause
    % reset to init position
    exetraj(reset_traj);
    disp('Reseted to initial position. Press keys to continue...')
    pause

end
%% run prediction

Ypred = cell(N_trajs,1);
for i=1:N_trajs
Ypred{i} = predict_traj(robot_model, x0, U{i}, t);
end
%% plot records
figure
title('recorded')
subplot(2,1,1)
hold on
for i=1:N_trajs
    plot(t, Y{i}(1,:) )
end
title('position')
legend('0','0.5','1')
hold off


subplot(2,1,2)
hold on
for i=1:N_trajs
    plot(t, Y{i}(2,:) )
end
title('rege power')
legend('0','0.5','1')
hold off
%% plot prediction
figure
title('prediction')
subplot(2,1,1)
hold on
for i=1:N_trajs
    plot(t, Ypred{i}(1,:) )
end
title('position')
legend('0','0.5','1')
hold off


subplot(2,1,2)
hold on
for i=1:N_trajs
    plot(t, Ypred{i}(end,:) )
end
title('rege power')
legend('0','0.5','1')
hold off


%% plot records
figure
subplot(2,2,1)
hold on
for i=1:N_trajs
    plot(t, Y{i}(1,:) )
end
title('position')
legend('0','0.5','1')
hold off


subplot(2,2,3)
hold on
for i=1:N_trajs
    plot(t, Y{i}(2,:) )
end
title('rege power')
legend('0','0.5','1')
hold off


subplot(2,2,2)
hold on
for i=1:N_trajs
    plot(t, Ypred{i}(1,:) )
end
title('position')
legend('0','0.5','1')
hold off


subplot(2,2,4)
hold on
for i=1:N_trajs
    plot(t, Ypred{i}(end,:) )
end
title('rege power')
legend('0','0.5','1')
hold off