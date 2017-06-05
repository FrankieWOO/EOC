%%%% step 1: define robot
% the maccepavd robot model has 8 state dimension
robot_model = Maccepavd1dof();

%%%% step 2: define task
%---- user specification ----%
target = pi/4;
T = 2;
dt = 0.02;
N = 101;
w = 0.1;
alpha = 1;
position0 = 0;
x0 = zeros(8,1); 
x0(1) = position0;
x0(3) = position0;
x0(6) = 0;

task_param = [];
task_param.target = target;
task_param.T = T;
task_param.w = w;
task_param.rege_ratio = alpha;
task_param.position0 =  position0;
task_param.x0 = x0; 
%define the initial spring pretension motor state
% all velocity, accel states are zero
task_param.with_motor_dynamics = 1;
task_param.multiple_inits = 0;
%----%

task = OptTask(robot_model, 'fast_reach', 'net_mechpower', task_param);
%%%%
costfn = CostMccvd1();


%%%% step 3: init OC handler
oc = ILQRController(robot_model, task);
oc.opt_param.online_plotting=0;

cost_param = [];
cost_param.w0 = 1;
cost_param.w = 0.1;
cost_param.alpha = 0;
cost_param.epsilon = 0;
cost_param.T = T;
cost_param.dt = dt;
cost_param.target = target;
cost_param.fd = 1; % use finite difference or not
f = @(x,u)robot_model.dynamics_with_jacobian(x,u);
j = @(x,u,t)costfn.j_reach_netmech(robot_model,x,u,t,cost_param);
%%
opt_param = [];
opt_param.umax = robot_model.umax;
opt_param.umin = robot_model.umin;
opt_param.lambda_init = 0.01;
opt_param.lambda_max  = 0.05;
opt_param.iter_max = 100;
opt_param.online_plotting = 0;
opt_param.online_printing = 1;
opt_param.dcost_converge = 10^-5;
opt_param.solver = 'rk4';
opt_param.target = target;
opt_param.umax = robot_model.umax;
opt_param.umin = robot_model.umin;
opt_param.T = T;
u0 = [cost_param.target; 0; 0];

result = ILQRController.ilqr(f, j, dt, N, x0, u0, opt_param);
%result = ILQRController.run_multiple(f, j, dt, N, x0, opt_param);

