% test a single trajectory

p.target = pi/3 ;
p.position0 = 0 ;
p.T = 2 ;

p.alpha = 0.5 ;
p.w = 0 ;
p.power_model = 8;
[u,x,L,cost,modelMaccepa] = optimize_mcc_vd_ilqr(p) ;

%%

Nt = size(x,2);
dt= p.T/Nt; t=(0:Nt-1)*dt;
settle_time = settling_time(p.x_target,x(1,:),x(2,:),t);
p.model=modelMaccepa;

plot_traj(x,u,p);
%% 
pp1 = p; pp2 = p;
pp1.recovery_ratio=0;
pp1.power_model = 8;

pp2.recovery_ratio=0.5;
pp2.power_model = 8;

power1=power_traj(x,u,pp1);
power2=power_traj(x,u,pp2);

costE1 = sum(power1)*dt
costE2 = sum(power2)*dt

%%
powers = cell(10,1);
for i=1:10
   p.power_model = i;
   powers{i} = power_traj(x,u,p); 
end

