% test optimization with same setting multiple times to see if the solution
% converge

x_target = pi/6 ; p.x_target = x_target ;
init_pos = 0; p.init_pos = init_pos ;
T = 2 ; p.T = T ;

recovery_ratio = 0.1 ; p.recovery_ratio = recovery_ratio ;
weight_energy = 0.1; p.weight_energy = weight_energy ;

Ntest = 10;
res_x = cell(Ntest,1);
res_u = cell(Ntest,1);
res_cost = cell(Ntest,1);
res_settlingTime = cell(Ntest,1);

for i =1:Ntest
    [res_u{i}, res_x{i}, res_cost{i}, modelMaccepa] = optimize_mcc_vd_ilqr(p) ;
    
    
end

p.model = modelMaccepa;
Nt = size(res_x{1},2);
dt= p.T/Nt; t=(0:Nt-1)*dt;
%%    
for j= 1:Ntest
    res_settlingTime{j} = settling_time(p.x_target,res_x{j}(1,:),res_x{j}(2,:),t);
end

