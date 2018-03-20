function [ p ] = calibrate_mccpvd( data )

%% load data
% M * qddot + D_f * qdot = Tau
% format long
% clear all
% %robot_model = Maccepavd1dof();
% act = ActMccpvd();
% 
% filelist = dir('newdata/*.csv');
% filelist = {filelist.name};
% nfiles = length(filelist);
% dataset = cellfun(@(x)readtable(x),fullfile('newdata',filelist),'UniformOutput',0);
% data = cell(size(dataset));
% for i = 1:nfiles
% data{i}.t = dataset{1}.(3);
% data{i}.q = dataset{1}.(5);
% data{i}.u1 = dataset{1}.(8);
% data{i}.u2 = dataset{1}.(9);
% end
% dataset2 = readtable('sysid2.csv');
% data2.t = dataset2.(3);
% data2.q = dataset2.(5);
% data2.u1 = dataset2.(8);
% data2.u2 = dataset2.(9);
% 
% dataset3 = readtable('sysid3.csv');
% data3.t = dataset3.(3);
% data3.q = dataset3.(5);
% data3.u1 = dataset3.(8);
% data3.u2 = dataset3.(9);
% 
% dataset4 = readtable('sysid4.csv');
% data4.t = dataset4.(3);
% data4.q = dataset4.(5);
% data4.u1 = dataset4.(8);
% data4.u2 = dataset4.(9);
%%
windowSize = 2; 
b = (1/windowSize)*ones(1,windowSize);
a = 1;
nfiles = length(data);
for i = 1:nfiles
    data{i}.q = filter(b, a, data{i}.q);
end
%%
%%windowSize = 4; 
%%b = (1/windowSize)*ones(1,windowSize);
%%a = 1;

for i = 1:nfiles
    data{i}.theta1 = data{i}.u1;
    data{i}.theta2 = data{i}.u2;
    %data{i}.dt = mean(diff(data{i}.t))*10^(-9);
    data{i}.dt = 0.015;
    data{i}.qdot = diff(data{i}.q)/data{i}.dt;
    %data{i}.qdot = filter(b,a,data{i}.qdot);
    data{i}.qddot = diff(data{i}.qdot)/data{i}.dt;
    %data{i}.qddot = filter(b,a,data{i}.qddot);
    data{i}.tau = act.torque_spring(data{i}.q,data{i}.theta1,data{i}.theta2);
end

%%

%C = [data1.qddot, data1.qdot(2:end)];
%d = data1.tau(3:end);
C =[];d=[];
for i =1:nfiles
   C = [C; data{i}.qddot, data{i}.qdot(2:end)];
   d = [d;data{i}.tau(3:end)];
end

p  = lsqnonneg(C,d);


end

