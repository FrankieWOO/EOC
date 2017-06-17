
file2read = 'recordtj.csv';
data = readtable(file2read);
datasim = load('tj45_1.mat');
tjsim = datasim.result;
%%
x = data.(5);
t = (0:length(x)-1)*0.006;
tsim = (0:length(tjsim.x)-1)*0.02;
plot(t,x)
hold on
plot(tsim,tjsim.x(1,:))
hold off