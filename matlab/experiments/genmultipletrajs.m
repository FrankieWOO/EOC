%function []=genmultipletrajs()
%% load data
datapath1 = 'trajs/constant/record/';
datapath2 = 'trajs/vardamp/record/';
datapath3 = 'trajs/optimal/record/';

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
ws1 = load('trajs/constant/constant.mat');
ws2 = load('trajs/vardamp/vardamp.mat');
ws3 = load('trajs/optimal/optimal.mat');

%%
for i=1:length(data1)
   
end

%% debug preprocess 
dp = DataProcess();
dd = dp.trim_head(data2{1});

window_size = 3;
%dd.p = dp.mavg_filter(dd.joint_position);
dd.p = dp.central_difference_smooth(dd.joint_position,5);
%dd.v1 = filter( (1/4)*ones(1,4), 1, [0; diff(dd.p)./diff(dd.header)]);
%dd.v2 = dp.compute_velocity_backdiff(dd.p, dd.header);
dd.v1 = dp.compute_velocity_centraldiff(dd.p, dd.header);
%dd.v2 = dp.compute_velocity_forwarddiff(dd.p, dd.header);
%dd.v3 = dp.central_difference_smooth(dd.v1,3);

dd.acc = dp.compute_accel_centraldiff(dd.v1, dd.header);

%dd.v3 = dp.exp_filter(dd.v1);
%dd.v4 = filter( (1/4)*ones(1,4), 1, dd.v2);
%dd.v4(1) = dd.v2(1);
figure
subplot(3,1,1)
plot(dd.header, dd.p)
grid on
subplot(3,1,2)
hold on
plot(dd.header, dd.v1)
%plot(dd.header, dd.v2)
%plot(dd.header, dd.v3)
grid on
%legend('1','2')
hold off
subplot(3,1,3)
plot(dd.header, dd.acc)
grid on

%%

dd.Erege = dp.compute_Erege(dd.rege_current, dd.header);
dd.power_rege=dp.compute_power_rege(dd.rege_current);
figure
hold on
plot(dd.header, dd.power_rege)
plot(ws2.traj_list{1}.tsim(1:end-1),ws2.traj_list{1}.power_rege)
hold off
figure
subplot(2,1,1)
hold on
plot(dd.header, dd.v)
plot(ws2.traj_list{1}.tsim, ws2.traj_list{1}.xsim(2,:))
hold off
subplot(2,1,2)
plot(ws2.traj_list{1}.t(1:end-1), ws2.traj_list{1}.u(3,:))

%% compute stats
% accuracy, avg settle time, rege E, input E


