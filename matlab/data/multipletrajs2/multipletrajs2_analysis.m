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
fprintf('---------| accuracy | accu. halt. | avg. settle time | Erege |\n')
fprintf('constant | %4.4f    | %4.8f | %4.4f            | %4.4f |\n',summary.data1.accuracy_score,summary.data1.accuracy_halt,summary.data1.avg_settle_time,summary.data1.total_Erege)
fprintf('vary damp| %4.4f    | %4.8f | %4.4f            | %4.4f |\n',summary.data2.accuracy_score,summary.data2.accuracy_halt,summary.data2.avg_settle_time,summary.data2.total_Erege)
fprintf('optimal  | %4.4f    | %4.8f | %4.4f            | %4.4f |\n',summary.data3.accuracy_score,summary.data3.accuracy_halt,summary.data3.avg_settle_time,summary.data3.total_Erege)

%% plot
DataProcess.plot_trajs(data1, ws1.target_list);
DataProcess.plot_trajs(data2, ws2.target_list);
DataProcess.plot_trajs(data3, ws3.target_list);

%% plot together

[p1,v1,acc1,t1,power_rege1,segments1,target_lines1]=DataProcess.merge_traj_list(data1(1:10), ws1.target_list(1:10), 1.5);
[p2,v2,acc2,t2,power_rege2,segments2,target_lines2]=DataProcess.merge_traj_list(data2(1:10), ws2.target_list(1:10), 1.5);
[p3,v3,acc3,t3,power_rege3,segments3,target_lines3]=DataProcess.merge_traj_list(data3(1:10), ws3.target_list(1:10), 1.5);

duration=1.5;
figure
subplot(3,1,1)
hold on
plot(t1, p1, 'LineWidth', 1.5)
plot(t2, p2, 'LineWidth', 1.5)
plot(t3, p3, 'LineWidth', 1.5)

plot(t1, target_lines1, '-.')
yL = get(gca, 'YLim');
for k=1:length(segments1)
    line([duration*k duration*k], yL, 'LineStyle','--','Color',[0.5 0.5 0.5])
end
xlabel('time (s)')
ylabel('position (rad)')
hold off

subplot(3,1,2)
hold on
plot(t1, v1,'LineWidth', 1.5)
plot(t2, v2,'LineWidth', 1.5)
plot(t3, v3,'LineWidth', 1.5)

yL = get(gca, 'YLim');
for k=1:length(segments1)
    line([duration*k duration*k], yL, 'LineStyle','--','Color',[0.5 0.5 0.5])
end
xlabel('time (s)')
ylabel('velocity (rad/s)')
hold off

subplot(3,1,3)
hold on
plot(t1, power_rege1,'LineWidth', 1)
plot(t2, power_rege2,'LineWidth', 1)
plot(t3, power_rege3,'LineWidth', 1)
yL = get(gca, 'YLim');
for k=1:length(segments1)
    line([duration*k duration*k], yL, 'LineStyle','--','Color',[0.5 0.5 0.5])
end
xlabel('time (s)')
ylabel('power (W)')
hold off


