%% train lwpr model to predict x_t+1 according to (x_t, u_t)
% discrete transition model
%% prepare data
%   state: q, qdot, theta1, theta2, theta1dot, theta2dot; action: u1, u2, u3


%collect_unit_samples
% U: cell list of control actions, Y: cell list of state records.
% process the dataset
Xtrain_aug = [];
Ytrain_aug = [];
n_test = 2;

indx_test = randi(length(Y),[1,n_test]);
indx_train = 1:length(Y);
indx_train(indx_test) = [];
Ytest = Y(indx_test);
Utest = U(indx_test);

Utrain = U(indx_train);
Ytrain = Y(indx_train);
%% simulate the motor trajectory by 2nd motor dynamics

sim_param.dt = 0.02;
sim_param.solver='rk4';
s_theta = [0;0;0;0];

f_motor = @()mccpvd_motor_dynamics ;

%%
for i=1:length(Ytrain)
     
    y = Ytrain{i};
    u = Utrain{i};
    for j=8:size(y,2)-1
        Xtrain_aug = [Xtrain_aug; y(1,j),y(2,j),u(:,j)'] ;
        Ytrain_aug = [Ytrain_aug; y(1,j+1)] ;
    end
    
end


%% initialize LWPR
% the model assumed here is discrete transition model.
model = lwpr_init(5,1,'name','dynamics_mccpvd');
model = lwpr_set(model,'update_D',0);
model = lwpr_set(model,'init_D',eye(5)*10);
model = lwpr_set(model,'init_alpha',ones(5)*250);
%model = lwpr_set(model,'w_gen',0.2);
%model = lwpr_set(model,'w_prune',0.7);   
model = lwpr_set(model,'diag_only',0);   
model = lwpr_set(model,'meta',1);
model = lwpr_set(model,'meta_rate',250);
model = lwpr_set(model,'kernel','Gaussian');   

%%%%%%%%%%%%%%%%%%%%%%%%
%  Transfer model into mex-internal storage
   model = lwpr_storage('Store', model);
%%%%%%%%%%%%%%%%%%%%%%%%

tic;

n_train = size(Xtrain_aug,1);
% train the model
for i_2=1:1
   inds = randperm(n_train);
   mse = 0;

   for i=1:n_train
	   [model,yp,w] = lwpr_update(model,Xtrain_aug(inds(i),:)',Ytrain_aug(inds(i),:)');
	   mse = mse + (Ytrain_aug(inds(i),:)-yp).^2;
   end

   nMSE = mse/n_train/var(Ytrain_aug,1);
   fprintf(1,'#Data=%d #rfs=%d nMSE=%5.3f\n',lwpr_num_data(model),lwpr_num_rfs(model),nMSE);   	
   if exist('fflush') % for Octave output only
      fflush(1);
   end   
end

toc

%% prediction of whole forward trajectory


Yp = cell(size(Ytest));

for i_test=1:length(Yp)
    ytest = Ytest{i_test};
    utest = Utest{i_test};
    xtest = [ytest(1:2,1:end-1); utest(:,:)];
    
    yp(1:2,1) = ytest(1:2,1);
    for k=1:size(ytest,2)-1
        xinput = [yp(:,k);xtest(3:end,k)];
        
        [res, w] = lwpr_predict(model, xinput,0.001);
        yp(:,k+1) = [res; res - yp(1,k)];
        
    end
    
    Yp{i_test} = yp;
end

figure
hold on
plot(ytest(1,:))
plot(yp(1,:))
hold off
legend('test traj','prediction')

%% create predictions for the test data
Yp = zeros(size(Ytest));
%for i=1:length(Xt),
%	yp = lwpr_predict(model,Xt(i,:)',0.001);
%	Yp(i,1) = yp;
%end
tic
% for k=1:10
%    yp = lwpr_predict(model,Xt',0.001);
% end
for i_1=1:length(Ytest)
	[yp,w]=lwpr_predict(model,Xtest(i_1,:)',0.001);
	Yp(i_1,1) = yp;
end

toc


ep   = Ytest-Yp;
mse  = mean(ep.^2);
nmse = mse/var(Ytest,1);
fprintf(1,'#nmse=%5.3f\n',nmse);


%%
model = lwpr_storage('GetFree',model);

