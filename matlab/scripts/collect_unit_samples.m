%% init ROS
init_ros

%% Define commands
N_trajs = 10;
N_t = 100;
U = cell(N_trajs,1);

for i=1:N_trajs
    u0= (rand(3,1)-[0.5; 0; 0]).*[2*pi/3; 2*pi/3; 1];
    u = repmat(u0, 1, N_t);
    U{i,1} = u; 
end

%% execute trajs
Y = exetrajs(U); 

