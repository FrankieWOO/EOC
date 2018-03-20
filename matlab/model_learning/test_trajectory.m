
%% execute example control command 
%load example_command.mat
uu = repmat([ 1;0.5;1],1,200);
n_command = size(uu,2);
x0 = [0;0] ; 
ps = []; ps.dt = 0.02 ; ps.solver ='rk4' ;

% define maccepa dynamic model
model_dynamic = model_maccepa('maccepa_model');
f = @(x, u) f_maccepa ( x, u, model_dynamic ); 
% simulate trajectory of underlying model
x_true_traj = simulate_feedforward(x0,f,uu,ps) ;

% simulate trajectory of learnt model
x_model_traj = zeros(2,n_command+1) ;
x_model_traj(:,1)= x0 ;
for i_1=1:n_command
    input = [x_model_traj(:,i_1);uu(:,i_1)] ;
    acc = lwpr_predict(model, input,0.001) ;
    x_model_traj(1,i_1+1) = x_model_traj(1,i_1) + x_model_traj(2,i_1)*ps.dt;
    x_model_traj(2,i_1+1) = x_model_traj(2,i_1) + acc*ps.dt ;
end
time_command = (0:n_command)*ps.dt;
plot(time_command,x_true_traj(1,:),'b-')
hold on
plot(time_command,x_model_traj(1,:),'r-')
legend('true','model') ;
hold off