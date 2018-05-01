%% init ROS
%init_ros
N_t = 100;
N_u=100;
reset0 = repmat( [0;0;0],1, N_u);
reset_traj_h = repmat([0; pi/2; 0], 1, N_u);
reset_traj = repmat([0; 0; 0], 1, N_u);
reset_traj = [reset_traj_h, reset_traj];
%% Define commands



u1s = linspace(-pi/4, pi/4, 6);
u2s = linspace(0, pi/2, 4);
N_trajs = length(u1s)*length(u2s);
U = cell(N_trajs,1);
k=1;
for i=1:length(u1s)
    for j=1:length(u2s)
        %u0 = (rand(3,1)-[0.5; 0; 0]).*[2*pi/3; 2*pi/3; 1];
        u0 = [u1s(i);u2s(j);0];
        u = repmat(u0, 1, N_t);
        U{k,1} = u;
        k=k+1;
    end
end

%% execute trajs
Y = cell(N_trajs,1);
for i=1:N_trajs

u = U{i};
y = exetraj(u);
y_rp = (y(2,:)/1000).^2 * 25;
y(2,:) = y_rp;
Y{i} = y;
    
    disp('Execution of traj done. Press keys to reset...')
    pause
    % reset to init position
    exetraj(reset_traj);
    disp('Reseted to initial position. Press keys to continue...')
    pause

end
