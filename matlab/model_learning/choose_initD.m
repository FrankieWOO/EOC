
Ds = 11:1:20;
ret = zeros(1,10);
load('sample.mat')
for i=1:10
   [~, ret(i)] = learn_model_maccepa(dataset,Ds(i));
end

