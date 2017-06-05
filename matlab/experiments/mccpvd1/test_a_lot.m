

% compare_weight2
% save('output_temp/compare_weight2_0.5')


%%
color1 = linspace(0,1,length(weights));

figure
hold on

load('w_alpha0.mat')

p.recovery_ratio = 0.5;
p.model = paras2{1}.model;
dt = 0.02;

work_net1 = zeros(51,1);
for i=1:51
    x = res_x{i};
    u = res_u{i};
    power = power_traj(x,u,p); 
    work_net1(i) = sum(power)*dt;
end
accuracy1 = res_cost_rt;

%options = fitoptions('Normalize', 'on', 'Robust', 'Bisquare')

f1 = fit(work_net1,accuracy1,'power2');

sh1 = scatter(work_net1,res_cost_rt,15,color1,'filled')
set(get(get(sh1,'Annotation'),'LegendInformation'),'IconDisplayStyle','off');
%plot(work_net,res_cost_rt)
h1=plot(f1,work_net1,accuracy1)
xlabel('cost of energy (J)')
ylabel('sum of task term cost')

load('w_alpha05.mat')
work_net2 = cell2mat(res_cost_e);
accuracy2 = res_cost_rt;
f2 = fit(work_net2,accuracy2,'power2');

sh2=scatter(cell2mat(res_cost_e),res_cost_rt,15,color1,'x')
set(get(get(sh2,'Annotation'),'LegendInformation'),'IconDisplayStyle','off');
h2=plot(f2,work_net2,accuracy2)
%plot(cell2mat(res_cost_e),res_cost_rt)
%legend([h1 h2],{'alpha=0','alpha=0.5'})
hold off