
% save('output_temp/compare_weight2_0.5')

alpha = [0,2,5,10];

runs = 10;
for k=1:runs
    for i=1:length(alpha)

    savepath = ['output_temp/optimization_weights/results_', num2str(alpha(i)),'_',num2str(k),'.mat'];
    opt_weight(alpha(i)/10,savepath);
    end
end

%%

for k=1:runs
    
   compare_weight3
   
   savepath = ['output_temp/compare_alpha/results_',num2str(k),'.mat'];
   
   save(savepath)
end
