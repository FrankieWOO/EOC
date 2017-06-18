format long
%---- user specification ----%
target = 0.5;
T = 2;
dt = 0.02;
N = T/dt + 1;
position0 = 0;
x0 = zeros(6,1); 
x0(1) = position0;
x0(3) = position0;
x0(5) = 0;

weights = linspace(0.0001,0.005,51);
Nw = 51;
ratio_loads = [0, 0.25, 0.5, 0.75, 1];
Nr = length(ratio_loads);
robot_params = cell(Nr);
robot_models = cell(Nr);
for i=1:Nr
    robot_params{i}.ratio_load = ratio_loads(i); 
    robot_models{i} = Mccpvd1dofModel();
    robot_models{i}.actuator = ActMccpvd(robot_params{i});

end

task_params = cell(Nw,Nr);
task_params2 = cell(Nw,Nr);
tasks = cell(Nw,Nr);
tasks2 = cell(Nw,Nr);
for i = 1:Nw
    for j = 1:Nr 
        task_params{i,j}.w = weights(i);
        task_params{i,j}.epsilon = 0;
        task_params{i,j}.T = T;
        task_params{i,j}.dt = dt;
        task_params{i,j}.target = target;
        task_params{i,j}.fd = 1; % use finite difference or not
        task_params{i,j}.x0 = x0;
        
        task_params{i,j}.f = @(x,u)robot_models{j}.dynamics_with_jacobian(x,u);
        
        task_params2{i,j}=task_params{i,j};
        task_params2{i,j}.w = task_params{i,j}.w*100;
        
        tasks{i,j} = mccpvd1_reach(robot_models{j}, task_params{i,j});
        tasks2{i,j} = mccpvd1_reach(robot_models{j}, task_params2{i,j});
        
        task_params{i,j}.j_netelec = @(x,u,t)tasks{i,j}.j_netelec(x,u,t);
        task_params{i,j}.j_elec = @(x,u,t)tasks{i,j}.j_elec(x,u,t);
        
        task_params2{i,j}.j_noutmech = @(x,u,t)tasks2{i,j}.j_noutmech(x,u,t);
        task_params2{i,j}.j_outmech = @(x,u,t)tasks2{i,j}.j_outmech(x,u,t);
        
    end
end



%%
opt_param = [];
opt_param.umax = robot_models{1}.umax;
opt_param.umin = robot_models{1}.umin;
opt_param.lambda_init = 0.01;
opt_param.lambda_max  = 0.5;
opt_param.iter_max = 100;
opt_param.online_plotting = 0;
opt_param.online_printing = 1;
opt_param.dcost_converge = 10^-6;
opt_param.solver = 'rk4';
opt_param.target = target;
opt_param.T = T;
opt_param.dt = dt;
% u0 can be full command sequence or just initial point
u0 = [target; 0; 0];
%u0 = [0; 0.1; 0];
results = cell(Nw,Nr);
parfor i = 1:Nw
    for j = 1:Nr
%result = ILQRController.ilqr(f, j, dt, N, x0, u0, opt_param);
        results{i,j}.result_elec_rege0 = ILQRController.ilqr(task_params{i,j}.f, task_params{i,j}.j_elec, dt, N, x0, u0, opt_param);
        results{i,j}.result_elec_rege1 = ILQRController.ilqr(task_params{i,j}.f, task_params{i,j}.j_netelec, dt, N, x0, u0, opt_param);
        results{i,j}.result_outmech_rege0 = ILQRController.ilqr(task_params2{i,j}.f, task_params2{i,j}.j_outmech, dt, N, x0, u0, opt_param);
        results{i,j}.result_outmech_rege1 = ILQRController.ilqr(task_params2{i,j}.f, task_params2{i,j}.j_noutmech, dt, N, x0, u0, opt_param);
        
    end
end


