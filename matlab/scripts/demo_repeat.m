%% init ROS
init_ros

%% Define commands
N_trajs = 4;
N_t = 100;
U = cell(N_trajs,1);

zero_traj = repmat([0;0;0],1,N_t);

zero_traj_h = repmat([0;0;1],1,N_t);
lowdamp = repmat( [-0.8;0;0],1, N_t);
middamp = repmat( [-0.8;0;0.5],1, N_t);
highdamp= repmat([-0.8;0;1],1,N_t);
U{1} = lowdamp;
U{2} = middamp;
U{3} = highdamp;
U{4} = zero_traj_h;

%% execute trajs
while true
%Y = exetrajs(U);
    for i=1:4
    exetraj(U{i});
    pause(2);
    end
end