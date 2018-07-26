clc
clear all

num_targets = 25;
init_position = 0;
joint_limit = [-pi/3, pi/3];
u2 = pi/6; % u2 fixed at pi/6
target_list = zeros(num_targets, 1);
traj_list = cell(num_targets,1);
% generate random targets
q0 = init_position;
for iter = 1:num_targets
    rd = random('unif', joint_limit(1), joint_limit(2));
    while abs(rd - q0)<pi/6
        rd = random('unif', joint_limit(1), joint_limit(2));
    end
    target_list(iter) = rd;
    traj = gentraj_vardamp(q0, rd, u2);
    traj_list{iter} = traj;
    q0 = rd; % assume we can get to the target
end

%%
folderpath = 'trajs/vardamp/';
for k = 1 : num_targets
    filename = [folderpath,'command/','traj',num2str(k),'.csv'];
    savetraj_to_csv(filename, traj_list{k});
end

save([folderpath, 'vardamp.mat'])

disp('saved')

%%
datapath = [folderpath,'record/'];

%%
files = dir([datapath,'/*.csv']);
data = cell(length(files),1);
for i=1:length(files)
    data{i,1} = table2struct(readtable([datapath,'record_traj',num2str(i),'.csv']),'ToScalar',true);
    
end
%%
q=[];
for i=1:length(data)
    q = [q; data{i}.joint_position];
end
plot(q)