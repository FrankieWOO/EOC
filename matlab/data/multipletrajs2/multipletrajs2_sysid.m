%% load data
cpath = pwd;
datapath1 = [cpath,'/trajs/constant/record/'];
datapath2 = [cpath,'/trajs/vardamp/record/'];
datapath3 = [cpath,'/trajs/optimal/record/'];

files1 = dir([datapath1,'/*.csv']);
data1 = cell(length(files1),1);
for i=1:length(files1)
    data1{i,1} = table2struct(readtable([datapath1,'record_traj',num2str(i),'.csv']),'ToScalar',true);
end

files2 = dir([datapath2,'/*.csv']);
data2 = cell(length(files2),1);
for i=1:length(files1)
    data2{i,1} = table2struct(readtable([datapath2,'record_traj',num2str(i),'.csv']),'ToScalar',true);
end

files3 = dir([datapath3,'/*.csv']);
data3 = cell(length(files3),1);
for i=1:length(files3)
    data3{i,1} = table2struct(readtable([datapath3,'record_traj',num2str(i),'.csv']),'ToScalar',true);
end
%%
ws1 = load([cpath,'/trajs/constant/constant.mat']);
ws2 = load([cpath,'/trajs/vardamp/vardamp.mat']);
ws3 = load([cpath,'/trajs/optimal/optimal.mat']);

%% process data
Data = [data1;data2;data3];
TargetList = [ws1.target_list;ws2.target_list;ws3.target_list];
TrajList = [ws1.traj_list;ws2.traj_list;ws3.traj_list];
for i = 1:length(Data)
    Data{i} = DataProcess.preprocess_single_traj(Data{i});
    Data{i}.accuracy_score = DataProcess.compute_accuracy_score(Data{i}.p, Data{i}.header, TargetList(i));
    Data{i}.accuracy_halt = DataProcess.compute_accuracy_after_halt(Data{i}, Data{i}.header, TargetList(i));
end

for i = 1:length(Data)
    crop_time = Data{i}.header <= Data{i}.settle_time;
    l = sum(crop_time);
    Data{i}.p = Data{i}.p(1:l);
    Data{i}.v = Data{i}.v(1:l);
    Data{i}.acc=Data{i}.acc(1:l);
    Data{i}.header=Data{i}.header(1:l);
end

for i=1:length(Data)
    Data{i}.m1 = convert_sequence_time(TrajList{i}.xsim(3,:), TrajList{i}.tsim, Data{i}.header);
    Data{i}.m2 = convert_sequence_time(TrajList{i}.xsim(4,:), TrajList{i}.tsim, Data{i}.header);
    Data{i}.u3 = convert_sequence_time(TrajList{i}.usim(3,:), TrajList{i}.tsim(1:end-1), Data{i}.header);
end

actuator = ActMccpvd();

for i=1:length(Data)
    Data{i}.tau = actuator.torque(Data{i}.p, Data{i}.v, Data{i}.m1, Data{i}.m2, Data{i}.u3);
end

data1 = Data(1:length(data1));
data2 = Data(length(data1)+1:length(data1)+length(data2));
data3 = Data(length(data1)+length(data2)+1:end);

%% debug preprocess 
dp = DataProcess();
dd = dp.trim_head(data3{2});
traj = ws3.traj_list{2};

%window_size = 3;
%dd.p = dp.mavg_filter(dd.joint_position);
%dd.p = dp.central_difference_smooth(dd.joint_position,5);
%dd.v1 = filter( (1/4)*ones(1,4), 1, [0; diff(dd.p)./diff(dd.header)]);
%dd.v2 = dp.compute_velocity_backdiff(dd.p, dd.header);
%dd.v = dp.compute_velocity_centraldiff(dd.p, dd.header);
%dd.v2 = dp.compute_velocity_forwarddiff(dd.p, dd.header);
%dd.v3 = dp.central_difference_smooth(dd.v1,3);

%dd.acc = dp.compute_accel_centraldiff(dd.v, dd.header);

%dd.v3 = dp.exp_filter(dd.v1);
%dd.v4 = filter( (1/4)*ones(1,4), 1, dd.v2);
%dd.v4(1) = dd.v2(1);
figure
subplot(5,1,1)
plot(dd.header, dd.p)
grid on
subplot(5,1,2)
hold on
plot(dd.header, dd.v)
%plot(dd.header, dd.v2)
%plot(dd.header, dd.v3)
grid on
%legend('1','2')
hold off
subplot(5,1,3)
plot(dd.header, dd.acc)
grid on

subplot(5,1,4)
hold on
plot(traj.tsim, traj.xsim(3,:))
plot(dd.header, dd.m1)
%plot(dd.header, dd.servo1_position)
hold off

subplot(5,1,5)
hold on
plot(traj.tsim, traj.xsim(4,:))
plot(dd.header, dd.m2)
hold off
%% 
X =[];Y=[];
for i =1:length(Data)
   X = [X; Data{i}.acc, Data{i}.v];
   Y = [Y;Data{i}.tau];
end

paras1  = lsqnonneg(X,Y);

X2 = [X, ones(size(X,1),1)];
paras2 = lsqnonneg(X2,Y);
%%
robot_param.inertia = paras1(1);
robot_param.Df = paras1(2);
robot = Mccpvd1dofModel(robot_param);
f = @(x,u)robot.dynamics_with_jacobian_fd(x,u);
simpara.dt = 0.001;
simpara.solver = 'rk4';
xsim = simulate_feedforward(traj.x(:,1),f,traj.usim,simpara);

%%
figure
hold on
plot(traj.tsim,traj.xsim(1,:))
plot(traj.tsim,xsim(1,:))
plot(dd.header, dd.p)
hold off
