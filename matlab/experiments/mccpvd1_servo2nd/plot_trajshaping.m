G=linspace(0,0.8,51); % Increase the .95 for lighter red at beginning, increase the 0 for lighter red at the end 
colors=[ones(51,1),G',G'];
%colors = [ linspace(1,0.5,51)', zeros(51,1),zeros(51,1) ];
figure
t = 0:0.02:2;
hold on
for i=1:51
   u = results{i,1}.result_mech_rege0.u;
   plot(t(1:end-1),u(1,:),'color',colors(i,:))
end

%%
data1 = load('wnr170619.mat');
data2 = load('wnr170620.mat');
data3 = load('wnr170621.mat');

%%
results_elec = cell(51,1);
results_load = cell(51,1);
results_mech = cell(51,1);
results_effort = cell(51,1);
tjf_elec = cell(51,1);
tjf_load = cell(51,1);
tjf_mech = cell(51,1);
tjf_effort = cell(51,1);
for i=1:51
    results_elec{i} = data1.results{i,1}.result_elec_rege0;
    results_load{i} = data1.results{i,1}.result_outmech_rege0;
    results_mech{i} = data2.results{i,1}.result_mech_rege0;
    results_effort{i} =data3.results{i,1}.result_effort_rege0;
    tjf_elec{i} = data1.results{i,1}.tjf_elec_rege0;
    tjf_load{i} = data1.results{i,1}.tjf_outmech_rege0;
    tjf_mech{i} = data2.results{i,1}.tjf_mech_rege0;
    tjf_effort{i} = data3.results{i,1}.tjf_effort_rege0;
    
end

%%
toplt = [1,5,10,15,20,25,30,35,40,45,50];
t = 0:0.02:2;
G=linspace(0,0.9,length(toplt)); % Increase the .95 for lighter red at beginning, increase the 0 for lighter red at the end 
colors=[G',G',G'];
reds = [ones(1,length(toplt))' ,G',G' ];
blues = [ G',G',ones(length(toplt),1) ]
figure
subplot(3,4,1)
hold on

for i=1:length(toplt)
   x = results_elec{toplt(i)}.x;
   plot(t,x(1,:),'color',blues(i,:))
end
hold off

subplot(3,4,5)
hold on
for i=1:length(toplt)
   u = results_elec{toplt(i)}.u;
   plot(t(1:end-1),u(1,:),'color',reds(i,:))
end
hold off

subplot(3,4,9)
hold on
for i=1:length(toplt)
   u = results_elec{toplt(i)}.u;
   plot(t(1:end-1),u(2,:),'color',reds(i,:))
end
hold off


subplot(3,4,2)
hold on

for i=1:length(toplt)
   x = results_mech{toplt(i)}.x;
   plot(t,x(1,:),'color',blues(i,:))
end
hold off

subplot(3,4,6)
hold on
for i=1:length(toplt)
   u = results_mech{toplt(i)}.u;
   plot(t(1:end-1),u(1,:),'color',reds(i,:))
end
hold off

subplot(3,4,10)
hold on
for i=1:length(toplt)
   u = results_mech{toplt(i)}.u;
   plot(t(1:end-1),u(2,:),'color',reds(i,:))
end
hold off


subplot(3,4,3)
hold on

for i=1:length(toplt)
   x = results_load{toplt(i)}.x;
   plot(t,x(1,:),'color',blues(i,:))
end
hold off

subplot(3,4,7)
hold on
for i=1:length(toplt)
   u = results_load{toplt(i)}.u;
   plot(t(1:end-1),u(1,:),'color',reds(i,:))
end
hold off

subplot(3,4,11)
hold on
for i=1:length(toplt)
   u = results_load{toplt(i)}.u;
   plot(t(1:end-1),u(2,:),'color',reds(i,:))
end
hold off


subplot(3,4,4)
hold on

for i=1:length(toplt)
   x = results_effort{toplt(i)}.x;
   plot(t,x(1,:),'color',blues(i,:))
end
hold off

subplot(3,4,8)
hold on
for i=1:length(toplt)
   u = results_effort{toplt(i)}.u;
   plot(t(1:end-1),u(1,:),'color',reds(i,:))
end
hold off

subplot(3,4,12)
hold on
for i=1:length(toplt)
   u = results_effort{toplt(i)}.u;
   plot(t(1:end-1),u(2,:),'color',reds(i,:))
end
hold off

%%
figure
for i=1:51
    energy_elec(i) = tjf_elec{i}.E_elec_posi;
    energy_mech(i) = tjf_mech{i}.E_elec_posi;
    energy_load(i) = tjf_load{i}.E_elec_posi;
    energy_effort(i) = tjf_effort{i}.E_elec_posi;
    ca_elec(i) = tjf_elec{i}.cost_accuracy;
    ca_mech(i) = tjf_mech{i}.cost_accuracy;
    ca_load(i) = tjf_load{i}.cost_accuracy;
    ca_effort(i) = tjf_effort{i}.cost_accuracy;
end
hold on
scatter(energy_elec,ca_elec)
scatter(energy_mech,ca_mech)
scatter(energy_load,ca_load)
scatter(energy_effort,ca_effort)
hold off





