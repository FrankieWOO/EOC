%function [ res ] = weightsNrobots()
%format long
%---- user specification ----%
target = 0.7;
T = 1;

dt = 0.02;
t = 0:dt:T;
N = T/dt + 1;
position0 = 0;
x0 = zeros(6,1); 
x0(1) = position0;
x0(3) = position0;
x0(5) = 0;

weights_elec = linspace(0,3*1e-3,51);
weights_effort = linspace(0,3,51);
Nw = 51;
%ratio_loads = [0, 0.25, 0.5, 0.75, 1];
ratio_loads = 1;

Nr = length(ratio_loads);
robot_params = cell(Nr);
robot_models = cell(Nr);


for i=1:Nr
   
    robot_params{i}.ratio_load = ratio_loads(i); 
    robot_params{i}.gear_d = 40;
    robot_models{i} = Mccpvd1dofModel(robot_params{i});
    robot_models{i}.actuator = ActMccpvd(robot_params{i});

end

task_params_elec = cell(Nw,Nr);
tasks_elec = cell(Nw,Nr);
task_params_effort = cell(Nw,Nr);
tasks_effort = cell(Nw,Nr);
for i = 1:Nw
    for j = 1:Nr 
        task_params_elec{i,j}.w_e = weights_elec(i);
        task_params_elec{i,j}.w_t = 1;
        task_params_elec{i,j}.w_tf = 1*dt;
        
        task_params_elec{i,j}.epsilon = 0;
        task_params_elec{i,j}.T = T;
        task_params_elec{i,j}.dt = dt;
        task_params_elec{i,j}.target = target;
        task_params_elec{i,j}.fd = 1; % use finite difference or not
        task_params_elec{i,j}.x0 = x0;
        
        task_params_elec{i,j}.f = @(x,u)robot_models{j}.dynamics_with_jacobian_fd(x,u);
        
        
        task_params_effort{i,j} = task_params_elec{i,j};
        task_params_effort{i,j}.w_e = weights_effort(i);
        task_params_effort{i,j}.w_r = task_params_effort{i,j}.w_e; 
        tasks_elec{i,j} = mccpvd1_reach(robot_models{j}, task_params_elec{i,j});
        
        tasks_effort{i,j} = mccpvd1_reach(robot_models{j}, task_params_effort{i,j});
        task_params_elec{i,j}.j_elec_rege = @(x,u,t)tasks_elec{i,j}.j_tf_elec_rege(x,u,t);
        task_params_elec{i,j}.j_elec = @(x,u,t)tasks_elec{i,j}.j_tf_elec(x,u,t);
        
        task_params_effort{i,j}.j_effort = @(x,u,t)tasks_effort{i,j}.j_tf_effort(x,u,t);
        task_params_effort{i,j}.j_effort_rege = @(x,u,t)tasks_effort{i,j}.j_tf_effort_rege(x,u,t);
    
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
for i = 1:Nw
   for j =1:Nr
        results{i,j}.result_elec = [];
        results{i,j}.result_elec_rege = [];
        
        results{i,j}.result_effort = [];
        results{i,j}.result_effort_rege=[];
        
        results{i,j}.tjf_elec = [];
        results{i,j}.tjf_elec_rege = [];
        
        results{i,j}.tjf_effort = [];
        results{i,j}.tjf_effort_rege=[];
   end
end
%%
 tsim = 0:0.001:T ;
        psim.solver = 'rk4';
        psim.dt = 0.001;
        
 paramtjf.target = target;
parfor i = 1:Nw
    for j = 1:Nr
%result = ILQRController.ilqr(f, j, dt, N, x0, u0, opt_param);
         


         results{i,j}.result_elec = ILQRController.run_multiple(task_params_elec{i,j}.f, task_params_elec{i,j}.j_elec, dt, N, x0, u0, opt_param);
         results{i,j}.result_elec_rege = ILQRController.run_multiple(task_params_elec{i,j}.f, task_params_elec{i,j}.j_elec_rege, dt, N, x0, u0, opt_param);
         %results{i,j}.result_outmech_rege0 = ILQRController.ilqr(task_params2{i,j}.f, task_params2{i,j}.j_outmech, dt, N, x0, u0, opt_param);
         %results{i,j}.result_outmech_rege1 = ILQRController.ilqr(task_params2{i,j}.f, task_params2{i,j}.j_noutmech, dt, N, x0, u0, opt_param);
         %results{i,j}.result_mech_rege0 =ILQRController.ilqr(task_params_elec{i,j}.f, task_params_elec{i,j}.j_mech, dt, N, x0, u0, opt_param);
         %results{i,j}.result_mech_rege1 =ILQRController.ilqr(task_params_elec{i,j}.f, task_params_elec{i,j}.j_netmech, dt, N, x0, u0, opt_param);
         
         results{i,j}.result_effort =ILQRController.run_multiple(task_params_effort{i,j}.f, task_params_effort{i,j}.j_effort, dt, N, x0, u0, opt_param);
         results{i,j}.result_effort_rege =ILQRController.run_multiple(task_params_effort{i,j}.f, task_params_effort{i,j}.j_effort_rege, dt, N, x0, u0, opt_param);
        
        
    end
end
%%
f = @(x,u)robot_models{1,1}.dynamics_with_jacobian_fd(x,u);
for i=1:Nw
    for j=1:Nr
    results{i,j}.result_elec.usim = scale_controlSeq(results{i,j}.result_elec.u,t,tsim);
    
    [results{i,j}.result_elec.xsim] = ...
        simulate_feedforward(x0,f,results{i,j}.result_elec.usim,psim);
    
    results{i,j}.result_elec_rege.usim = scale_controlSeq(results{i,j}.result_elec_rege.u,t,tsim);
    
    [results{i,j}.result_elec_rege.xsim] = ...
        simulate_feedforward(x0,f,results{i,j}.result_elec_rege.usim,psim);
    
    
        results{i,j}.result_effort.usim = scale_controlSeq(results{i,j}.result_effort.u,t,tsim);
    
    [results{i,j}.result_effort.xsim] = ...
        simulate_feedforward(x0,f,results{i,j}.result_effort.usim,psim);
    
    results{i,j}.result_effort_rege.usim = scale_controlSeq(results{i,j}.result_effort_rege.u,t,tsim);
    
    [results{i,j}.result_effort_rege.xsim] = ...
        simulate_feedforward(x0,f,results{i,j}.result_effort_rege.usim,psim);
    
    
    results{i,j}.tjf_elec = traj_features(robot_models{1,1}, results{i,j}.result_elec.xsim, results{i,j}.result_elec.usim, 0.001,paramtjf);
    results{i,j}.tjf_elec_rege = traj_features(robot_models{1,1}, results{i,j}.result_elec_rege.xsim, results{i,j}.result_elec_rege.usim, 0.001,paramtjf);
    
    results{i,j}.tjf_effort = traj_features(robot_models{1,1}, results{i,j}.result_effort.xsim, results{i,j}.result_effort.usim, 0.001,paramtjf);
    results{i,j}.tjf_effort_rege = traj_features(robot_models{1,1}, results{i,j}.result_effort_rege.xsim, results{i,j}.result_effort_rege.usim, 0.001,paramtjf);

    end
end


%%
% res.robot_models = robot_models;
% res.results = results;
% res.task_params = task_params;
% res.task_params2 = task_params2;
% 
% end