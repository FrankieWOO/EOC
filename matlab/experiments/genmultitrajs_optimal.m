clc
clear all

num_targets = 25;
init_position = 0;
joint_limit = [-pi/3, pi/3];

target_list = zeros(num_targets, 1);
traj_list = cell(num_targets,1);
% generate random targets
q0 = init_position;
x0 = [q0; 0; 0; pi/6; 0; 0];
for iter = 1:num_targets
    rd = random('unif', joint_limit(1), joint_limit(2));
    while abs(rd - q0)<pi/6
        rd = random('unif', joint_limit(1), joint_limit(2));
    end
    target_list(iter) = rd;
    traj = gentraj_optimal(x0, rd, 5e2);
    traj_list{iter} = traj;
    q0 = rd; % assume we can get to the target
    x0 = traj.xsim(:,end);
end
%%

for iter = 1:length(traj_list)
    traj_list{iter}.u(1:2,:) = traj_list{iter}.x(3:4,1:end-1);
end


%%
folderpath = 'trajs/optimal/';
for k = 1 : num_targets
    filename = [folderpath,'command/','traj',num2str(k),'.csv'];
    savetraj_to_csv(filename, traj_list{k});
end

save([folderpath, 'optimal.mat'])

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