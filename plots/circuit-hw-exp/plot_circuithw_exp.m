data1 = readtable('damping.csv');
data2 = readtable('Prege.csv');
%%
Mdamping = table2array(data1(:,4:13));

Mp = table2array(data2(:,4:13));
u = table2array(data1(:,1));

mean_damp = table2array(data1(:,14));
mean_P = table2array(data2(:,14));
%%
figure
subplot(2,1,1)
hold on
for i=1:10
scatter(u,Mdamping(:,i),20,'k','filled')
end
plot(u,mean_damp)
hold off

subplot(2,1,2)
hold on
for i=1:10
scatter(u,Mp(:,i),20,'k','filled')
end
plot(u,mean_P)
hold off