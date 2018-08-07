%function []=genmultipletrajs()
%% load data
cpath = pwd;
datapath1 = [cpath,'/trajs/constant/record/'];
datapath2 = [cpath,'/trajs/vardamp/record/'];
datapath3 = [cpath,'/trajs/fixdamp/record/'];
datapath4 = [cpath,'/trajs/optimal/record/'];

Num_targets = 25;
Num_trials = 10;

%files1 = dir([datapath1,'/*.csv']);
data1 = cell(Num_targets,Num_trials);
for i=1:Num_targets
    for j=1:Num_trials
    data1{i,j} = table2struct(readtable([datapath1,'record_traj',num2str(i),'_',num2str(j),'.csv']),'ToScalar',true);
    end
end

%files2 = dir([datapath2,'/*.csv']);
data2 = cell(Num_targets,Num_trials);
for i=1:Num_targets
    for j=1:Num_trials
    data2{i,j} = table2struct(readtable([datapath2,'record_traj',num2str(i),'_',num2str(j),'.csv']),'ToScalar',true);
    end
end

%files3 = dir([datapath3,'/*.csv']);
data3 = cell(Num_targets,Num_trials);
for i=1:Num_targets
    for j=1:Num_trials
    data3{i,j} = table2struct(readtable([datapath3,'record_traj',num2str(i),'_',num2str(j),'.csv']),'ToScalar',true);
    end
end

data4 = cell(Num_targets,Num_trials);
for i=1:Num_targets
    for j=1:Num_trials
    data4{i,j} = table2struct(readtable([datapath4,'record_traj',num2str(i),'_',num2str(j),'.csv']),'ToScalar',true);
    end
end
%%
ws1 = load([cpath,'/trajs/constant/constant.mat']);
ws2 = load([cpath,'/trajs/vardamp/vardamp.mat']);
ws3 = load([cpath,'/trajs/fixdamp/fixdamp.mat']);
ws4 = load([cpath,'/trajs/optimal/optimal.mat']);

%% process data
inertia = 3.6e-3;
Data = [data1;data2;data3;data4];
TargetList = [ws1.target_list;ws2.target_list;ws3.target_list;ws4.target_list];
for i = 1:size(Data,1)
    for j = 1:size(Data,2)
    Data{i,j} = DataProcess.preprocess_single_traj2(Data{i,j});
    Data{i,j}.accuracy_score = DataProcess.compute_accuracy_score(Data{i,j}.p, Data{i,j}.header, TargetList(i));
    Data{i,j}.accuracy_halt = DataProcess.compute_accuracy_after_halt(Data{i,j}, TargetList(i));
    Data{i,j}.overshot = DataProcess.compute_overshot(Data{i,j}, TargetList(i));
    Data{i,j}.max_kinetic = inertia*(max(abs(Data{i,j}.v)))^2/2;
    end
end
data1 = Data(1:size(data1,1),:);
data2 = Data(size(data1,1)+1:size(data1,1)+size(data2,1),:);
data3 = Data(size(data1,1)+size(data2,1)+1:75,:);
data4 = Data(76:end,:);


%% compute smmary stats
summary.data1 = DataProcess.compute_trajs_stats(data1);
summary.data2 = DataProcess.compute_trajs_stats(data2);
summary.data3 = DataProcess.compute_trajs_stats(data3);
summary.data4 = DataProcess.compute_trajs_stats(data4);

%%
fprintf('---------| accuracy | accu. halt. | overshot | avg. settle time | Erege | Kinetic | pcnt |\n')
fprintf('constant | %4.4f    | %4.8f | %4.4f| %4.4f | %4.4f |%4.4f |%4.4f |\n',summary.data1.accuracy_score,summary.data1.accuracy_halt,summary.data1.avg_overshot,summary.data1.avg_settle_time,summary.data1.avg_Erege,summary.data1.avg_kinetic,summary.data1.pcnt_Erege)
fprintf('vary damp| %4.4f    | %4.8f | %4.4f| %4.4f | %4.4f |%4.4f |%4.4f |\n',summary.data2.accuracy_score,summary.data2.accuracy_halt,summary.data2.avg_overshot,summary.data2.avg_settle_time,summary.data2.avg_Erege,summary.data2.avg_kinetic,summary.data2.pcnt_Erege)
fprintf('fix damp | %4.4f    | %4.8f | %4.4f| %4.4f | %4.4f |%4.4f |%4.4f |\n',summary.data3.accuracy_score,summary.data3.accuracy_halt,summary.data3.avg_overshot,summary.data3.avg_settle_time,summary.data3.avg_Erege,summary.data3.avg_kinetic,summary.data3.pcnt_Erege)
fprintf('optimal  | %4.4f    | %4.8f | %4.4f| %4.4f | %4.4f |%4.4f |%4.4f |\n',summary.data4.accuracy_score,summary.data4.accuracy_halt,summary.data4.avg_overshot,summary.data4.avg_settle_time,summary.data4.avg_Erege,summary.data4.avg_kinetic,summary.data4.pcnt_Erege)

fileID = fopen('multireach_summary.csv','w');
fprintf(fileID,'experiment, overshot, settle time, rege energy, Kinetic, pcnt \n');
fprintf(fileID,'constant, %4.4f, %4.4f, %4.4f, %4.4f, %4.4f \n',summary.data1.avg_overshot,summary.data1.avg_settle_time,summary.data1.avg_Erege,summary.data1.avg_kinetic,summary.data1.pcnt_Erege*100);
fprintf(fileID,'vary damp, %4.4f, %4.4f, %4.4f, %4.4f, %4.4f \n',summary.data2.avg_overshot,summary.data2.avg_settle_time,summary.data2.avg_Erege,summary.data2.avg_kinetic,summary.data2.pcnt_Erege*100);
fprintf(fileID,'fix damp, %4.4f, %4.4f, %4.4f, %4.4f, %4.4f \n',summary.data3.avg_overshot,summary.data3.avg_settle_time,summary.data3.avg_Erege,summary.data3.avg_kinetic,summary.data3.pcnt_Erege*100);
fprintf(fileID,'optimal, %4.4f, %4.4f, %4.4f, %4.4f, %4.4f \n',summary.data4.avg_overshot,summary.data4.avg_settle_time,summary.data4.avg_Erege,summary.data4.avg_kinetic,summary.data4.pcnt_Erege*100);

fclose(fileID);
%% plot
DataProcess.plot_trajs(data1, ws1.target_list);
DataProcess.plot_trajs(data2, ws2.target_list);
DataProcess.plot_trajs(data3, ws3.target_list);

%% plot together
ntrajs2plot = 5;
[p1,v1,acc1,t1,power_rege1,segments1,target_lines1]=DataProcess.merge_traj_list(data1(1:ntrajs2plot,1), ws1.target_list(1:ntrajs2plot), 1.5);
[p2,v2,acc2,t2,power_rege2,segments2,target_lines2]=DataProcess.merge_traj_list(data2(1:ntrajs2plot,1), ws2.target_list(1:ntrajs2plot), 1.5);
[p3,v3,acc3,t3,power_rege3,segments3,target_lines3]=DataProcess.merge_traj_list(data3(1:ntrajs2plot,1), ws3.target_list(1:ntrajs2plot), 1.5);
[p4,v4,acc4,t4,power_rege4,segments4,target_lines4]=DataProcess.merge_traj_list(data4(1:ntrajs2plot,1), ws4.target_list(1:ntrajs2plot), 1.5);

duration=1.5;
figure
subplot(3,1,1)
hold on
plot(t1, p1, 'LineWidth', 1.5, 'LineStyle','--')
plot(t2, p2, 'LineWidth', 1.5)
plot(t3, p3, 'LineWidth', 2, 'LineStyle','--', 'Color', [1,0.6,0.6])
plot(t4, p4, 'LineWidth', 1.5)
legend('const.','var. damp','fix. damp','optimal')
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
plot(t1, v1,'LineWidth', 1.5, 'LineStyle','--')
plot(t2, v2,'LineWidth', 1.5)
plot(t3, v3,'LineWidth', 2, 'LineStyle','--', 'Color', [1,0.6,0.6])
plot(t4, v4,'LineWidth', 1.5)

yL = get(gca, 'YLim');
for k=1:length(segments1)
    line([duration*k duration*k], yL, 'LineStyle','--','Color',[0.5 0.5 0.5])
end
xlabel('time (s)')
ylabel('velocity (rad/s)')
hold off

subplot(3,1,3)
hold on
plot(t1, power_rege1,'LineWidth', 1, 'LineStyle','--')
plot(t2, power_rege2,'LineWidth', 1)
plot(t3, power_rege3,'LineWidth', 1, 'LineStyle','--', 'Color', [1,0.6,0.6])
plot(t4, power_rege4,'LineWidth', 1)

yL = get(gca, 'YLim');
for k=1:length(segments1)
    line([duration*k duration*k], yL, 'LineStyle','--','Color',[0.5 0.5 0.5])
end
xlabel('time (s)')
ylabel('power (W)')
hold off


