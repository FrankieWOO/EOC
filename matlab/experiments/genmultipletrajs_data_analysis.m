%function []=genmultipletrajs()
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
for i = 1:length(Data)
    Data{i} = DataProcess.preprocess_single_traj(Data{i});
    Data{i}.accuracy_score = DataProcess.compute_accuracy_score(Data{i}.p, Data{i}.header, TargetList(i));
    Data{i}.accuracy_halt = DataProcess.compute_accuracy_after_halt(Data{i}, Data{i}.header, TargetList(i));
end

data1 = Data(1:length(data1));
data2 = Data(length(data1)+1:length(data1)+length(data2));
data3 = Data(length(data1)+length(data2)+1:end);

%% compute smmary stats
summary.data1 = DataProcess.compute_trajs_stats(data1);
summary.data2 = DataProcess.compute_trajs_stats(data2);
summary.data3 = DataProcess.compute_trajs_stats(data3);
%%
fprintf('---------| accuracy | avg. settle time | Erege |\n')
fprintf('constant | %4.4f    | %4.4f            | %4.4f |\n',summary.data1.accuracy_score,summary.data1.avg_settle_time,summary.data1.total_Erege)
fprintf('vary damp| %4.4f    | %4.4f            | %4.4f |\n',summary.data2.accuracy_score,summary.data2.avg_settle_time,summary.data2.total_Erege)
fprintf('optimal  | %4.4f    | %4.4f            | %4.4f |\n',summary.data3.accuracy_score,summary.data3.avg_settle_time,summary.data3.total_Erege)

%% plot
DataProcess.plot_trajs(data1, ws1.target_list);
DataProcess.plot_trajs(data2, ws2.target_list);
DataProcess.plot_trajs(data3, ws3.target_list);


%% debug preprocess 
dp = DataProcess();
%dd = dp.trim_head(data2{10});
dd = Data{35};
traj = ws2.traj_list{10};

window_size = 3;
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
plot(traj.tsim(1:end-1), traj.usim(1,:))
plot(dd.header, dd.servo1_position)
hold off

subplot(5,1,5)
hold on
plot(traj.tsim, traj.xsim(4,:))
plot(traj.tsim(1:end-1), traj.usim(2,:))
hold off
%%

dd.Erege = dp.compute_Erege(dd.rege_current, dd.header);
dd.power_rege=dp.compute_power_rege(dd.rege_current);
figure
subplot(2,1,1)
hold on
title('rege power')
plot(dd.header, dd.power_rege)
plot(traj.tsim(1:end-1),traj.power_rege)
legend('recorded','predicted')
hold off
subplot(2,1,2)
hold on
title('velocity')
plot(dd.header, dd.v)
plot(traj.tsim, traj.xsim(2,:))
legend('recorded','predicted')
ylabel('velocity')
hold off


figure
subplot(2,1,1)
hold on
plot(dd.header, dd.v)
plot(traj.tsim, traj.xsim(2,:))
legend('recorded','predicted')

hold off
subplot(2,1,2)
plot(traj.t(1:end-1), traj.u(3,:))

%%

dd.accuracy_score = dp.compute_accuracy_score(dd.p,dd.header,-0.7812)


%% compute stats
% accuracy, avg settle time, rege E, input E





