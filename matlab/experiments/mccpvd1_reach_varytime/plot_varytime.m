data1 = load('vt_results_elec.mat');
data2 = load('vt_results_elec_rege.mat');
results_elec = data1.results_elec;
results_elec_rege = data2.results_elec_rege;

%%

Nres = length(results_elec);
Tfs = zeros(Nres,1);
eff_elec = zeros(Nres,1);
eff_elec_rege = zeros(Nres,1);
E_elec = zeros(Nres,1);
E_elec_rege = zeros(Nres,1);
for i=1:Nres
    Tfs(i) = size(results_elec{i,1}.traj.u,2)*0.02;
    eff_elec(i) = results_elec{i,1}.tjf.rege_ratio;
    eff_elec_rege(i) = results_elec_rege{i,1}.tjf.rege_ratio;
    E_elec(i) = results_elec{i,1}.tjf.E_netelec;
    E_elec_rege(i) = results_elec_rege{i,1}.tjf.E_netelec;
    
end

figure
hold on
%plot(Tfs,eff_elec,Tfs,eff_elec_rege)
plot(Tfs,E_elec,Tfs,E_elec_rege)

hold off
