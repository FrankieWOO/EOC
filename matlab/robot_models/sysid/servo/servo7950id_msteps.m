%% load m steps data
dataset = readtable('hitec7950/msteps_nodelay.csv');
%data.t = dataset.(3)*10^-9;
x = dataset.(3);
u = dataset.(4);
x = x - x(1); % offset calibration error

%% 3rd order
dt = 0.006; %1/163.436;
xdot = diff(x)/dt;
xddot = diff(xdot)/dt;
xdddot = diff(xddot)/dt;

z1 = u-x; z1 = z1(4:end);
z2 = -xdot(3:end);
z3 = -xddot(2:end);

Z = [z1,z2,z3];Z=-Z;

p_n = inv((Z'*Z))*Z'*xdddot;
%% 2nd order least square




%%
opt_param=[];
opt_param.solver = 'rk4';
opt_param.dt = dt;

p_wada = [14000;1150;37.5];
f_wada = @(x,u)servo_3rdmodel(x,u,p_wada);
usim = u';
tjsim_wada = simulate_feedforward(zeros(3,1),f_wada,usim,opt_param);

p_sim = [4376.65, 497.57, 24.29]; % p_sim is the paras estimated from simulink
f = @(x,u)servo_3rdmodel(x,u,p_sim);
%usim = ones(1,100)*u(1);
tjsim3 = simulate_feedforward(zeros(3,1),f,usim,opt_param);

p_2nd = 40;
f_2nd=@(x,u)servo_2ndmodel(x,u,p_2nd);
tjsim_2nd = simulate_feedforward(zeros(2,1),f_2nd,usim,opt_param);

figure
hold on
plot(x)
plot(u)
plot(tjsim3(1,:))
plot(tjsim_wada(1,:))
plot(tjsim_2nd(1,:))
legend('measurement','input','3rd','Wada','2nd')
hold off

%%
N1 = 50;
N2 = 1000;
tsim1 = 0:1/N1:1;
u1 = 0.5*ones(1,N1);
tsim2 = 0:1/N2:1;
u2 = 0.5*ones(1,N2);
option1.solver = 'euler';
option1.dt = 1/N1;
option2.solver = 'rk4';
option2.dt = 1/N2;

option3= option1; option3.solver = 'ode45';
tj2_1 = simulate_feedforward(zeros(2,1),f_2nd,u1,option1);
tj2_2 = simulate_feedforward(zeros(2,1),f_2nd,u2,option2);
tj2_3 = simulate_feedforward(zeros(2,1),f_2nd,u1,option3);
figure
hold on
plot(tsim1,tj2_1(1,:))
%plot(tsim2,tj2_2(1,:))
plot(tsim1,tj2_3(1,:))
hold off


