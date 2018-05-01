%% init ros
% after initialisation in the global variables there should be mccssr_raw
% subscribed to '/sensors_raw'
% init_ros
global rate mccssr_raw pubcmd cmdmsg
%% calibrate the joint sensor

disp('place the joint in zero position, press key to record')
pause

for i=1:100
    ssrdata = receive(mccssr_raw, 1);
    x(i) = ssrdata.JointSensor;
end

x0 = mean(x);

disp('command to +60 deg, press key to record')
exetraj([pi/3;0.6;0]);
pause
for i=1:100
    ssrdata = receive(mccssr_raw,1);
    x(i) = ssrdata.JointSensor;
end

xp60 = mean(x);

disp('command to -60 deg, press key to record')
exetraj([-pi/3;0.6;0]);
pause
for i=1:100
    ssrdata = receive(mccssr_raw,1);
    x(i) = ssrdata.JointSensor;
end

xn60 = mean(x);

joint_weight = 2*pi/3/(xp60-xn60);
joint_offset = -joint_weight*x0;

%% calibrate the servo 1 sensor

disp('place the joint in zero position, press key to record')
exetraj([0;0.6;0])
pause

for i=1:100
    ssrdata = receive(mccssr_raw, 1);
    x(i) = ssrdata.Servo1Sensor;
end

x0 = mean(x);

disp('command to +60 deg, press key to record')
exetraj([pi/3;0.6;0]);
pause
for i=1:100
    ssrdata = receive(mccssr_raw,1);
    x(i) = ssrdata.Servo1Sensor;
end

xp60 = mean(x);

disp('command to -60 deg, press key to record')
exetraj([-pi/3;0.6;0]);
pause
for i=1:100
    ssrdata = receive(mccssr_raw,1);
    x(i) = ssrdata.Servo1Sensor;
end

xn60 = mean(x);

servo1_weight = 2*pi/3/(xp60-xn60);
servo1_offset = -servo1_weight*x0;
%%

