
Ts = 0.4:0.02:2;
amplitude = 0.7;
results = cell(length(Ts),1);

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
robot_param.inertia_l = 0.0016;
%robot_param.Df = 0.01;
robot_model = Mccpvd1dofModel(robot_param);
robot_model.actuator = ActMccpvd(param_act);

for i = 1:length(Ts)

T = Ts(i);
freq = 1/T;
dt = 0.02;
Nu = floor(T/dt);
N = Nu + 1;

t = 0:dt:T;
[p_ref, v_ref] = mccpvd1_trackperiod.generate_simpletraj(amplitude, freq, t);
x_ref = [p_ref;v_ref];
x0 = [x_ref(:,1); x_ref(1,1); 0;0;0];

task_param = [];
task_param.x_ref = x_ref;
task_param.T = T;
task_param.dt = dt;
task_param.freq = freq;
task_param.Nu = Nu;
task_param.w_e = 0;
task_param.w_t = 1 ;
%task_param.w_tf= 1*dt ;
task_param.w_r = task_param.w_e*1 ;
task = mccpvd1_trackperiod(robot_model, task_param);

f = @(x,u)robot_model.dynamics_with_jacobian_fd(x,u) ;

j = @(x,u,t)task.j_effort(x,u,t);

%%
opt_param = [];
opt_param.umax = robot_model.umax;
opt_param.umin = robot_model.umin;
opt_param.lambda_init = 0.01;
opt_param.lambda_max  = 2000;
opt_param.iter_max = 100;
opt_param.online_plotting = 0;
opt_param.online_printing = 1;
opt_param.dcost_converge = 10^-6;
opt_param.solver = 'rk4';

opt_param.T = T;

% u0 can be full command sequence or just initial point
%u0 = [task_param.target; 0; 0];
u0 = [0; 0; 0];

result = ILQRController.ilqr(f, j, dt, N, x0, u0, opt_param);
results{i,1}.traj = result;
%result2 = ILQRController.ilqr_sim(f, j2, dt, N, x0, u0, opt_param);
tjfparam.x_ref = x_ref; tjfparam.T = T;
tjf = mccpvd1_trackperiod.traj_features(robot_model, result.x, result.u, dt, tjfparam);
results{i,1}.tjf = tjf;
%% run controller on plant
N_run = 5;

ppi = []; ppi.xn = [repmat(result.x(:,1:end-1),1,N_run), result.x(:,end)]; 
ppi.un = repmat(result.u, 1, N_run); ppi.Ln = repmat(result.L, [1 1 N_run]);
ppi.umax = robot_model.umax; ppi.umin = robot_model.umin;
policy = @(x,n)pi_ilqr(x, n, ppi);
ps_feedback.solver = 'rk4'; ps_feedback.dt = dt; ps_feedback.N = Nu*N_run+1;
[x_simpi, u_simpi] = simulate_feedback_time_indexed ( x0, f, policy, ps_feedback );
t_simpi = 0:dt:(T*N_run);
x_ref_simpi = [repmat(x_ref(:,1:end-1),1,N_run), x_ref(:,end)];

results{i,1}.ppi = ppi;
results{i,1}.x_simpi = x_simpi;
results{i,1}.u_simpi = u_simpi;

tjfparam_Nrun.x_ref = x_ref_simpi; tjfparam_Nrun.T = T*N_run;
tjf_Nrun = mccpvd1_trackperiod.traj_features(robot_model, x_simpi, u_simpi, dt, tjfparam_Nrun);
results{i,1}.tjf_Nrun = tjf_Nrun;
end

