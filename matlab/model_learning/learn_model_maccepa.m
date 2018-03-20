function [ model, nMSE ] = learn_model_maccepa( dataset,initD )
%LEARN_MODEL_MACCEPA Summary of this function goes here
%   Detailed explanation goes here
Xtrain = dataset(:,1:5) ; Ytrain = dataset(:,6) ;
n_train = size(Xtrain,1) ; 
%% learn model
% initialize LWPR
model = lwpr_init(5,1,'name','learn_dynamics');
model = lwpr_set(model,'update_D',0);
model = lwpr_set(model,'init_D',eye(5)*initD);
model = lwpr_set(model,'init_alpha',ones(5)*25);
%model = lwpr_set(model,'w_gen',0.2);
%model = lwpr_set(model,'w_prune',0.7);   
model = lwpr_set(model,'diag_only',0);   
model = lwpr_set(model,'meta',1);
model = lwpr_set(model,'meta_rate',250);
model = lwpr_set(model,'kernel','Gaussian');   

%%%%%%%%%%%%%%%%%%%%%%%%
%  Transfer model into mex-internal storage
   model = lwpr_storage('Store',model);
%%%%%%%%%%%%%%%%%%%%%%%%

tic;

% train the model
for i_2=1:1
   inds = randperm(n_train);
   mse = 0;

   for i=1:n_train,
	   [model,yp,~] = lwpr_update(model,Xtrain(inds(i),:)',Ytrain(inds(i),:)');
	   mse = mse + (Ytrain(inds(i),:)-yp).^2;
   end

   nMSE = mse/n_train/var(Ytrain,1);
   fprintf(1,'#Data=%d #rfs=%d nMSE=%5.3f\n',lwpr_num_data(model),lwpr_num_rfs(model),nMSE);   	
%    if exist('fflush') % for Octave output only
%       fflush(1);
%    end   
end
toc

model = lwpr_storage('GetFree',model);

end

