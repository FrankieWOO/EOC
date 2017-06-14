

filelist = dir('hitec7950/*.csv');
filelist = {filelist.name};
nfiles = length(filelist);

%read data into cell arracy
dataset = cellfun(@(x)csvread(x,1,0),fullfile('hitec7950',filelist),'UniformOutput',0);
% t, position, input
%%
data = cell(size(dataset));
%data = dataset{7};
%sys = fitparam(data);
for i = 1:length(data)
   data{i}.t = dataset{i}(:,1);
   data{i}.dt = (diffdata{i}.t);
   data{i}.x = dataset{i}(:,2);
end
%%
t = data(1:end,1); t = t-t(1);
tdot = t(2:end) - t(1:end-1);
x_origin = data(1:end,2);
u = data(1:end,3);

%% load m steps data
dataset = readtable('hitec7950/msteps.csv');
%data.t = dataset.(3)*10^-9;
x = dataset.(3);
u = dataset.(4);
x = x - x(1); % offset calibration error

%% filter data
windowSize = 4; 
b = (1/windowSize)*ones(1,windowSize);
a = 1;
x = filter(b,a,x_origin);
%%
xdot = (x(2:end)-x(1:end-1))./tdot;
xddot = (xdot(2:end)-xdot(1:end-1))./tdot(2:end);
xdddot = (xddot(2:end)-xddot(1:end-1))./tdot(3:end);

z1 = u-x; z1 = z1(4:end);
z2 = -xdot(3:end);
z3 = -xddot(2:end);

Z = [z1,z2,z3];Z=-Z;
%% least square optimizer
[p,resnorm,residual,exitflag,output,lambda] = lsqlin(Z,xdddot);
%% normal equation
p_n = inv((Z'*Z))*Z'*xdddot;

%%
opt_param=[];
opt_param.solver = 'rk4';
Nt=1000;
opt_param.dt = (t(end)-t(1))/100;

p_wada = [14000;1150;37.5];
f_wada = @(x,u)servo_3rdmodel(x,u,p_wada);
usim = ones(1,100)*u(1);
tjsim_wada = simulate_feedforward(zeros(3,1),f_wada,usim,opt_param);

f = @(x,u)servo_3rdmodel(x,u,p_n);
usim = ones(1,100)*u(1);
tjsim = simulate_feedforward(zeros(3,1),f,usim,opt_param);
figure
hold on
plot(t,x)
plot((0:100)*opt_param.dt,tjsim(1,:))
plot((0:100)*opt_param.dt,tjsim_wada(1,:))
hold off

%%
xdot = (x(3:end)-x(1:end-2))./(tdot(1:end-1)+tdot(2:end) );