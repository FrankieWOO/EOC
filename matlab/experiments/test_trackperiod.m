%% optimise a single period to track a given trajectory.
% use feedback control to make the tracking converge after few periods.


amplitude = 0.7;
T = 1.2;
freq = 1/T;
dt = 0.02;
Nu = round(T/dt);
N = Nu+1;

t = 0:dt:T;
[p_ref, v_ref] = mccpvd1_trackperiod.generate_simpletraj(amplitude,freq,t);
x_ref = [p_ref;v_ref];
x0 = [x_ref(:,1); x_ref(1,1); 0;0;0];

param_act.ratio_load = 1;
param_act.gear_d = 100;
%param_act.Kd = 0.0212;
%param_act.K1 = 1;
%param_act.K2 = 1;
%param_act.R1 = 0.5;
%param_act.R2 = 0.5;
%param_act.Ks = 500;
param_act.J1 = 0.001;
param_act.J2 = 0.001;
param_act.D1 = 0.02;
param_act.D2 = 0.02;
robot_param.inertia_l = 0.0016;
robot_param.has_gravity = true;
%robot_param.Df = 0.01;
robot_model = Mccpvd1dofModel(robot_param, param_act);

task_param = [];
task_param.x_ref = x_ref;
task_param.T = T;
task_param.dt = dt;
task_param.freq = freq;
task_param.Nu = Nu;
task_param.w_e = 1e-6 ;
task_param.w_t = 100 ;
%task_param.w_tf= 1*dt ;
task_param.w_r = task_param.w_e*0;
task = mccpvd1_trackperiod(robot_model, task_param);

f = @(x,u)robot_model.dynamics_with_jacobian_fd(x,u) ;

j = @(x,u,t)task.j_effort_rege(x,u,t);

%j1 = @(x,u,t)task1.j_effort(x,u,t);
%j2 = @(x,u,t)task1.j_effort_rege(x,u,t);




%%
opt_param = [];
opt_param.umax = robot_model.umax;
opt_param.umin = robot_model.umin;
opt_param.lambda_init = 0.1;
opt_param.lambda_max  = 500000;
opt_param.iter_max = 250;
opt_param.online_plotting = 0;
opt_param.online_printing = 1;
opt_param.dcost_converge = 10^-8;
opt_param.solver = 'rk4';

opt_param.T = T;

% u0 can be full command sequence or just initial point
%u0 = [task_param.target; 0; 0];
u0 = [0; 0; 0];

result = ILQRController.ilqr(f, j, dt, N, x0, u0, opt_param);
%result2 = ILQRController.ilqr_sim(f, j2, dt, N, x0, u0, opt_param);
%%
% figure
% subplot(2,2,1)
% plot(t, result.x(1,:),t,x_ref(1,:))
% axis([0 T -inf inf ])
% legend('q','q_ref')
% xlabel('time (sec)')
% subplot(2,2,2)
% plot(t(1:end-1), result.u(1,:), t, result.x(3,:))
% legend('u1','motor1')
% xlabel('time (sec)')
% axis([0 T -inf inf ])
% subplot(2,2,3)
% plot(t(1:end-1), result.u(2,:), t, result.x(4,:))
% legend('u2','motor2')
% xlabel('time (sec)')
% axis([0 T -inf inf ])
% subplot(2,2,4)
% plot(t(1:end-1), result.u(3,:))
% legend('u3')
% xlabel('time (sec)')
% axis([0 T -inf inf ])
%%
% run controller on plant
N_run = 10;


ppi = []; ppi.xn = [repmat(result.x(:,1:end-1),1,N_run), result.x(:,end)]; 
ppi.un = repmat(result.u, 1, N_run); ppi.Ln = repmat(result.L, [1 1 N_run]);
ppi.umax = robot_model.umax; ppi.umin = robot_model.umin;
policy = @(x,n)pi_ilqr(x, n, ppi);
ps_feedback.solver = 'rk4'; ps_feedback.dt = dt; ps_feedback.N = Nu*N_run+1;
[x_simpi, u_simpi] = simulate_feedback_time_indexed ( x0, f, policy, ps_feedback );
t_simpi = 0:dt:(T*N_run);
x_ref_simpi = [repmat(x_ref(:,1:end-1),1,N_run), x_ref(:,end)];

%%

figure
hold on
plot(t_simpi, x_simpi(1,:))
plot(t_simpi, x_ref_simpi(1,:))
hold off

%%
tjfparam.x_ref = x_ref; tjfparam.T = T;
tjf = mccpvd1_trackperiod.traj_features(robot_model, result.x, result.u, dt, tjfparam);
%tjf.E_elec
%tjf.E_netelec
%tjf.E_effort
%tjf.rege_ratio


