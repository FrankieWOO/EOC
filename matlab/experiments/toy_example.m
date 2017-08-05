
target = pi/3;
T = 1;
dt = 0.02;
N = T/dt + 1;
t = 0:dt:T;
parm = [];
parm.has_gravity = false;
pact = [];
pact.max_stiffness = 100;
pact.max_damping = 100;

robot = Pendulum1(parm,pact);
%%%% create task
ptask = [];
ptask.target = target;
ptask.w_t = 1;
ptask.w_e = 1;
ptask.w_tf = 1;
ptask.w_r = 0;
ptask.dt = dt;
task = pendulum1_reach(ptask);
%%
f = @(x,u)robot.dynamics(x,u);
j1 = @(x,u,t)task.j_fastreach(x,u,t);

%%
psim.dt = 0.02;
psim.solver = 'rk4';

result0.k = 81;
result0.d = 2*sqrt(result0.k);
result0.u = repmat( [target; result0.k/pact.max_stiffness; result0.d/pact.max_damping], 1, N-1 );
result0.x = simulate_feedforward(x0,f,result0.u,psim);
%%
opt_param = [];
opt_param.umax = robot.umax;
opt_param.umin = robot.umin;
opt_param.lambda_init = 0.05;
opt_param.lambda_max  = 5000000;
opt_param.iter_max = 250;
opt_param.online_plotting = 0;
opt_param.online_printing = 1;
opt_param.dcost_converge = 10^-8;
opt_param.solver = 'rk4';
opt_param.target = target;

opt_param.T = T;

x0 = [0;0];
% u0 can be full command sequence or just initial point
%u0 = result0.u;
u0 = [0; 0.1; 0];

result1 = ILQRController.ilqr(f, j1, dt, N, x0, u0, opt_param);
figure
plot(t, result0.x(1,:))
hold on
plot(t, ones(1,N)*target)
plot(t, result1.x(1,:))