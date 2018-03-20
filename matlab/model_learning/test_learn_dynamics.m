
%% define hardware model and true dynamics
% model
model_dynamic = model_maccepa('maccepa_model');

% dynamics
f = @(x, u) f_maccepa ( x, u, model_dynamic ); 
%%
umax = [ pi/2 ; pi/2 ; 1 ] ;
umin = [ -pi/2 ; 0 ; 0 ] ;

% time step of each trajectory. 
nt = 25 ;

%% simulate trajectroies to generate dataset
x0s = [0 pi/4 -pi/4 pi/2 -pi/2; 0 0 0 0 0] ;
%x0 = [0 ; 0] ;
n_inits = size(x0s,2) ;
ps = []; ps.solver ='rk4'; ps.dt = 0.02 ;
n_u1 = 50 ; n_u2 = 25 ; n_u3 = 10; ratio_test = 1/5;
u1_mesh = linspace( umin(1) , umax(1) , n_u1 ) ;
u2_mesh = linspace( umin(2) , umax(2) , n_u2 ) ;
u3_mesh = linspace( umin(3) , umax(3) , n_u3 ) ;
n_dataset= n_u1*n_u2*n_u3*nt*n_inits ;
dataset = zeros( n_dataset , 6 ) ;
for i_1 = 1:n_u1
   for i_2 = 1:n_u2
      for i_3 = 1:n_u3
          for i_4 = 1:n_inits
              x0 = x0s(:,i_4) ;
              u1_seq = ones(1,nt)*u1_mesh(i_1) ;
              u2_seq = ones(1,nt)*u2_mesh(i_2) ;
              u3_seq = ones(1,nt)*u3_mesh(i_3) ;
              u_seq = [u1_seq; u2_seq; u3_seq] ;
              x_seq = simulate_feedforward(x0,f,u_seq,ps);
              xddot = zeros(1,nt);
              for i_t=1:nt
                  xddot(i_t) = qddot_maccepa(x_seq(1,i_t), x_seq(2,i_t), u_seq(:,i_t), model_dynamic);
              end
              index_start = 1 + nt*(i_4-1) + nt*n_inits*(i_3-1) + ...
                  nt*n_inits*n_u3*(i_2-1) + nt*n_inits*n_u3*n_u2*(i_1-1) ;
              index_end  = 25 + nt*(i_4-1) + nt*n_inits*(i_3-1) + ...
                  nt*n_inits*n_u3*(i_2-1) + nt*n_inits*n_u3*n_u2*(i_1-1) ;
              dataset(index_start:index_end,:) = ...
                [ x_seq(1,1:end-1)', x_seq(2,1:end-1)', u1_seq',u2_seq',u3_seq',xddot'];
          end
      end
   end
end

%% split dataset
% test dataset
n_test = round( ratio_test*n_dataset ) ;

inds_test = randperm(n_dataset,n_test) ;

Xtest = dataset(inds_test,1:5) ; Ytest = dataset(inds_test,6);

% training dataset
dataset(inds_test,:) = [] ;
Xtrain = dataset(:,1:5) ; Ytrain = dataset(:,6) ;
n_train = size(Xtrain,1) ; 



% 
% for i_1=1:n_inits
%     x0 = x0s(:,i_1);
%     u1_seq = -pi/2 + pi.*rand(1,nt);
%     u2_seq = rand(1,nt).*(pi/2) ;
%     u3_seq = rand(1,nt);
%     u_seq = [u1_seq; u2_seq; u3_seq] ;
% 
% 
%     x_seq = simulate_feedforward(x0,f,u_seq,ps);
%     xddot = zeros(1,nt);
%     for i_1=1:nt
%         xddot(i_1) = qddot_maccepa(x_seq(1,i_1), x_seq(2,i_1), u_seq(:,i_1), model_dynamic);
%     end
% 
%     dataset = [ x_seq(1,1:end-1)', x_seq(2,1:end-1)', u1_seq',u2_seq',u3_seq',xddot'];
% 
%     for nrep=1:499
%         u1_seq = -pi/2 + pi.*rand(1,nt);
%         u2_seq = rand(1,nt).*(pi/2) ;
%         u3_seq = rand(1,nt);
%         u_seq = [u1_seq; u2_seq; u3_seq] ;
% 
%         x_seq = simulate_feedforward(x0,f,u_seq,ps);
%         for i_1=1:nt
%         xddot(i_1) = qddot_maccepa(x_seq(1,i_1), x_seq(2,i_1), u_seq(:,i_1), model_dynamic);
%         end
%     newdata = [ x_seq(1,1:end-1)', x_seq(2,1:end-1)', u1_seq',u2_seq',u3_seq',xddot'] ;
%     dataset = [dataset; newdata] ;
%     end
% end
% 
% n = 100000;
% u1 = -pi/2 + pi.*rand(n,1);
% u2 = rand(n,1).*(pi/2) ;
% u3 = rand(n,1);
% x1 = -pi/2 + pi.*rand(n,1);
% x2 = rand(n,1).*15 ;
% xddot = zeros(n,1);
% for i=1:n
%    xddot(i) = qddot_maccepa(x1(i),x2(i),[u1(i);u2(i);u3(i)],model_dynamic) ;
% end
% dataset = [x1,x2,u1,u2,u3,xddot];
% training dataset



%% learn model
% initialize LWPR
model = lwpr_init(5,1,'name','learn_dynamics');
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
   model = lwpr_storage('Store',model);
%%%%%%%%%%%%%%%%%%%%%%%%

tic;

% train the model
for i_2=1:1
   inds = randperm(n_train);
   mse = 0;

   for i=1:n_train,
	   [model,yp,w] = lwpr_update(model,Xtrain(inds(i),:)',Ytrain(inds(i),:)');
	   mse = mse + (Ytrain(inds(i),:)-yp).^2;
   end

   nMSE = mse/n_train/var(Ytrain,1);
   fprintf(1,'#Data=%d #rfs=%d nMSE=%5.3f\n',lwpr_num_data(model),lwpr_num_rfs(model),nMSE);   	
   if exist('fflush') % for Octave output only
      fflush(1);
   end   
end
toc


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
for i_1=1:length(Ytest),
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

