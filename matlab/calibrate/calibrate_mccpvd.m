function [ p ] = calibrate_mccpvd( data )
% data: cell array of trajectory data. Each traj is a struct containing q,
% u1, u2, u3, t, 
%% load data
% M * qddot + D_f * qdot = Tau
%% apply filter on recorded position
%windowSize = 2; 
%b = (1/windowSize)*ones(1,windowSize);
%a = 1;
%nfiles = length(data);
%for i = 1:nfiles
%    data{i}.q = filter(b, a, data{i}.q);
%end
%%
%%windowSize = 4; 
%%b = (1/windowSize)*ones(1,windowSize);
%%a = 1;
dt = 0.02;
nsamples = length(data);
for i = 1:nsamples
    
    %data{i}.dt = mean(diff(data{i}.t))*10^(-9);
    %data{i}.dt = 0.02;
    data{i}.qdot = diff(data{i}.q)/dt;
    %data{i}.qdot = filter(b,a,data{i}.qdot);
    data{i}.qddot = diff(data{i}.qdot)/dt;
    %data{i}.qddot = filter(b,a,data{i}.qddot);
    %data{i}.tau = act.torque_spring(data{i}.q,data{i}.theta1,data{i}.theta2);
end

%% merge data into matrix C,d

%C = [data1.qddot, data1.qdot(2:end)];
%d = data1.tau(3:end);
C =[];d=[];
for i =1:nsamples
   C = [C; data{i}.qddot', data{i}.qdot(2:end)'];
   d = [d;data{i}.tau(3:end)'];
end
%% solve least square by linear algebra
p  = lsqnonneg(C,d);


end

