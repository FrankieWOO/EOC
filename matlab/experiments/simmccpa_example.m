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
robot = Mccpvd1dofModel(robot_param, param_act);

target = pi/3;
T = 1;
dt = 0.02;
N = T/dt + 1;
t = 0:dt:T;

%%%% create task
ptask = [];
ptask.target = target;
ptask.w_t = 1000;
ptask.w_e = 1;
ptask.w_tf = 1000;
ptask.w_r = 0;
ptask.dt = dt;
task = pendulum1_reach(ptask);
x0 = [0;0];
f = @(x,u)robot.dynamics(x,u);
j1 = @(x,u,t)task.j_fastreach(x,u,t);

%% critical damped
psim.dt = 0.02;
psim.solver = 'rk4';

result0.u2 = 0.2;
result0.d = 2*sqrt(result0.k);
result0.u = repmat( [target; result0.k/pact.max_stiffness; result0.d/pact.max_damping], 1, N-1 );
result0.x = simulate_feedforward(x0,f,result0.u,psim);


%% dynamic braking & hybrid mode
opt_param = [];
opt_param.umax = robot.umax;
opt_param.umin = robot.umin;
opt_param.lambda_init = 0.5;
opt_param.lambda_max  = 1e10;
opt_param.iter_max = 300;
opt_param.online_plotting = 0;
opt_param.online_printing = 1;
opt_param.dcost_converge = 1e-9;
opt_param.solver = 'rk4';
opt_param.target = target;

opt_param.T = T;

% u0 can be full command sequence or just initial point
%u0 = result0.u;
u0 = [target; 0; 0];

%result1 = ILQRController.ilqr(f, j1, dt, N, x0, u0, opt_param);
%% pure regenerative braking
u3_max = robot.actuator.u_max_regedamp;
opt_param2 = opt_param;
opt_param2.umax(3) = u3_max;
result2 = ILQRController.ilqr(f, j1, dt, N, x0, u0, opt_param2);
result1 = ILQRController.ilqr(f, j1, dt, N, x0, u0, opt_param);

%% fixed max rege power damping

%%
figure
subplot(2,2,1)
plot(t, result0.x(1,:))
hold on
%plot(t, ones(1,N)*target)
plot(t, result1.x(1,:))
plot(t, result2.x(1,:))

hold off

subplot(2,2,2)
hold on
plot(t(1:end-1), result0.u(1,:))
plot(t(1:end-1), result1.u(1,:))
plot(t(1:end-1), result2.u(1,:))
hold off


subplot(2,2,3)
hold on
plot(t(1:end-1), result0.u(2,:)*robot.actuator.max_stiffness)
plot(t(1:end-1), result1.u(2,:)*robot.actuator.max_stiffness)
plot(t(1:end-1), result2.u(2,:)*robot.actuator.max_stiffness)
hold off

subplot(2,2,4)
hold on
plot(t(1:end-1), result0.u(3,:)*robot.actuator.max_damping)
plot(t(1:end-1), result1.u(3,:)*robot.actuator.max_damping)
plot(t(1:end-1), result2.u(3,:)*robot.actuator.max_damping)

%%
uu3 = 0:0.01:1;
for i=1:length(uu3)
    dd(i) = robot.actuator.damping(uu3(i));
    pp(i) = robot.actuator.power_rege(1,uu3(i));
end
figure
plot(uu3, dd, uu3, pp)

