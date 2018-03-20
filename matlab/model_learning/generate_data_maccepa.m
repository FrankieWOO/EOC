function [ dataset ] = generate_data_maccepa(  )
%% define hardware model and true dynamics

% parameters setting

% what percetage of dataset is used for test data
% ratio_test = 1/5;

% time step of each trajectory. 
nt = 25 ;

ps = []; ps.solver ='rk4'; ps.dt = 0.02 ;

n_u1 = 50 ; n_u2 = 25 ; n_u3 = 10; 

x0s = [0 pi/4 -pi/4 pi/2 -pi/2; 0 0 0 0 0] ;

% model
model_dynamic = model_maccepa('maccepa_model');

% dynamics
f = @(x, u) f_maccepa ( x, u, model_dynamic ); 
umax = [ pi/2 ; pi/2 ; 1 ] ;
umin = [ -pi/2 ; 0 ; 0 ] ;



%% simulate trajectroies to generate dataset

%x0 = [0 ; 0] ;
n_inits = size(x0s,2) ;

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

end