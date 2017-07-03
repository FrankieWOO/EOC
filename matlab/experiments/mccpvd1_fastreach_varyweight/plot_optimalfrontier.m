data = load('data_fastreach_varyweight.mat');
results = data.results;
tasks_effort = data.tasks_effort;
tasks_elec = data.tasks_elec;

%%

E_elec = zeros( length(tasks_elec),1 );
E_elec_rege = zeros( length(tasks_elec),1 );
A_elec = zeros( length(tasks_elec),1 );
A_elec_rege = zeros( length(tasks_elec),1 );
E_effort = zeros( length(tasks_elec),1 );
E_effort_rege = zeros( length(tasks_elec),1 );
A_effort = zeros( length(tasks_elec),1 );
A_effort_rege = zeros( length(tasks_elec),1 );
for i = 1:length(tasks_elec)
    E_elec(i) = results{i,1}.tjf_elec.E_netelec;
    E_elec_rege(i) = results{i,1}.tjf_elec_rege.E_netelec;
    E_effort(i) = results{i,1}.tjf_effort.E_netelec;
    E_effort_rege(i) = results{i,1}.tjf_effort_rege.E_netelec;
    A_elec(i) = results{i,1}.tjf_elec.cost_accuracy;
    A_elec_rege(i) = results{i,1}.tjf_elec_rege.cost_accuracy;
    A_effort(i) = results{i,1}.tjf_effort.cost_accuracy;
    A_effort_rege(i) = results{i,1}.tjf_effort_rege.cost_accuracy;
end

figure
hold on
%scatter(E_elec(8:end),A_elec(8:end))
%scatter(E_elec_rege(8:end),A_elec_rege(8:end))
scatter(E_effort(2:20),A_effort(2:20))
scatter(E_effort_rege(2:20),A_effort_rege(2:20))
hold off

%%
w_elec = zeros(length(tasks_elec),1);
eff_elec = zeros(length(tasks_elec),1);
eff_elec_rege = zeros(length(tasks_elec),1);
w_effort = zeros(length(tasks_elec),1);
eff_effort = zeros(length(tasks_elec),1);
eff_effort_rege = zeros(length(tasks_elec),1);
for i=1:length(tasks_elec)
    w_elec(i) = tasks_elec{i,1}.w_e;
    eff_elec(i) = results{i,1}.tjf_elec.rege_ratio;
    eff_elec_rege(i) = results{i,1}.tjf_elec_rege.rege_ratio;
    w_effort(i) = tasks_effort{i,1}.w_e;
    eff_effort(i) = results{i,1}.tjf_effort.rege_ratio;
    eff_effort_rege(i) = results{i,1}.tjf_effort_rege.rege_ratio;
end
figure
%plot(w_elec,eff_elec,w_elec,eff_elec_rege)

plot(w_effort,eff_effort,w_effort,eff_effort_rege)
