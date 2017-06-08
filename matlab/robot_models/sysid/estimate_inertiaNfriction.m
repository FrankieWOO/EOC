
%% load data
% M * qddot + D_f * qdot = Tau
robot_model = Maccepavd1dof();

dataset1 = readtable('sysid1.csv');
data1.t = dataset1.(3);
data1.q = dataset1.(5);
data1.u1 = dataset1.(8);
data1.u2 = dataset1.(9);

dataset2 = readtable('sysid2.csv');
data2.t = dataset2.(3);
data2.q = dataset2.(5);
data2.u1 = dataset2.(8);
data2.u2 = dataset2.(9);

dataset3 = readtable('sysid3.csv');
data3.t = dataset3.(3);
data3.q = dataset3.(5);
data3.u1 = dataset3.(8);
data3.u2 = dataset3.(9);

dataset4 = readtable('sysid4.csv');
data4.t = dataset4.(3);
data4.q = dataset4.(5);
data4.u1 = dataset4.(8);
data4.u2 = dataset4.(9);


%%
data1.theta1 = data1.u1;
data1.theta2 = data1.u2;
qdot = diff(data1.q)/0.00612;
