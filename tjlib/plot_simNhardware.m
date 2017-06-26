format long
file2read = 'recordtj_effort.csv';
data = readtable(file2read);
datasim = load('tjexample_effort.mat');
tjsim = datasim.result;
x = data.(5);
xoffset = x(1);
x = x-xoffset;

%%
file2read2 = 'recordtj_elec.csv';
data2 = readtable(file2read2);
datasim2 = load('tjexample_elec.mat');
tjsim2 = datasim2.result;
x2 = data2.(5);
xoffset2 = x2(1);
x2 = x2 - xoffset2;

%%
t = (0:length(x)-1)/75;
tsim = (0:length(tjsim.x)-1)*0.02;
figure
plot(t,x)
hold on
plot(tsim,tjsim.x(1,:))
hold off
%%
figure
plot(t,x2)
hold on
plot(tsim,tjsim2.x(1,:))
hold off

%%
pelec1_effort = abs(data.(9));
pelec2_effort = abs(data.(10));
pelec1_elec = abs( data2.(9));
pelec2_elec = abs( data2.(10));
pelec_effort = pelec1_effort+pelec2_effort;
pelec_elec = pelec1_elec + pelec2_elec;

figure
hold on
plot(t,pelec_effort, t,pelec_elec)
legend('Minimal effort','Minimal elec')
hold off
%%
xdot = [0;diff(x)];
xdot2 =[0;diff(x2)];

windowSize = 3; 
b = (1/windowSize)*ones(1,windowSize);
a = 1;
xdot = filter(b,a,xdot);
xdot2= filter(b,a,xdot2);

Vemf = robot_model.actuator.gear_d*robot_model.actuator.Kd*xdot;
I1 = data.(8);
I2 = data2.(8);
Vemf2 = robot_model.actuator.gear_d*robot_model.actuator.Kd*xdot2;

power_rege = Vemf.*I1;
power_rege2= Vemf.*I2;
E_rege1 = sum(power_rege)/75;
E_rege2 = sum(power_rege2)/75;
%%
figure
subplot(5,1,1)
plot(t,xdot)

subplot(5,1,2)
plot(tsim(1:end-1),tjsim.u(1:2,:))
subplot(5,1,3)
plot(tsim(1:end-1),tjsim.u(3,:))

subplot(5,1,4)
plot(t,I1)
subplot(5,1,5)
plot(t,power_rege)
%%
figure
subplot(5,1,1)
plot(t,xdot2)

subplot(5,1,2)
plot(tsim(1:end-1),tjsim2.u(1:2,:))
subplot(5,1,3)
plot(tsim(1:end-1),tjsim2.u(3,:))

subplot(5,1,4)
plot(t,I2)
subplot(5,1,5)
plot(t,power_rege2)


%%
E_effort = sum(pelec_effort)*7/75;
E_elec = sum(pelec_elec)*7/75;

