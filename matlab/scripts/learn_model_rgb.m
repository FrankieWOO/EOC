% learn rigid-body dynamics model
%% collect data
collect_unit_samples

%% specify robot model
robot_param=[];
param_act.ratio_load = 1;
param_act.gear_d = 40;
%robot_param.inertia = 0.00315;
%robot_param.Df = 0.0062; %seems 0.0023 is too small for swing from 0.
robot_param.inertia = 0.0015;
robot_param.Df = 0.004; 

robot_model = Mccpvd1dofModel(robot_param, param_act);
%% process data U and Y
x0=zeros(6,1);
Nsamples = length(U);
data = cell(Nsamples,1);
t = 0:0.02:2;
for i=1:Nsamples
    data_i.u1 = U{i}(1,:);
    data_i.u2 = U{i}(2,:);
    data_i.u3 = U{i}(3,:);
    data_i.q = Y{i}(1,:);
    data_i.rc = Y{i}(2,:);
    ypred = predict_traj(robot_model, x0, U{i}, t);
    data_i.theta1 = ypred(3,:);
    data_i.theta2 = ypred(4,:);
    
    data_i.tau = robot_model.actuator.torque_spring(data_i.q,data_i.theta1,data_i.theta2);
    data{i}=data_i;
end

%%
parampred = calibrate_mccpvd(data);

%%
robot_param.inertia = 0.0006;
robot_param.Df = 0.0104;
robot_model = Mccpvd1dofModel(robot_param, param_act);
