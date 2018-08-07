

if ~exist('target_list','var')
    genmultipletrajs_random_targets
end
%%

u3 = 0.5; % u2 fixed at pi/6
if ~exist('wr','var')
    wr = 0;
end
traj_list = cell(num_targets,1);
% generate random targets
%q0 = init_position;
x0 = [init_position; 0; init_position; pi/6; 0; 0];
for iter = 1:length(target_list)
    
    traj = gentraj_fixdamp(x0, target_list(iter), u3, wr);
    traj_list{iter} = traj;
    x0 = traj.xsim(:,end); % assume we can get to the target
end
%% use simulated motor trajectory as desired tracking trajectory for servos

for iter = 1:length(traj_list)
    traj_list{iter}.u(1:2,:) = traj_list{iter}.x(3:4,1:end-1);
end

%%
folderpath = 'trajs/fixdamp/';
for k = 1 : num_targets
    filename = [folderpath,'command/','traj',num2str(k),'.csv'];
    savetraj_to_csv(filename, traj_list{k});
end

save([folderpath, 'fixdamp.mat'])

disp('saved fixdamp')

%%
% datapath = [folderpath,'record/'];
% 
% %%
% files = dir([datapath,'/*.csv']);
% data = cell(length(files),1);
% for i=1:length(files)
%     data{i,1} = table2struct(readtable([datapath,'record_traj',num2str(i),'.csv']),'ToScalar',true);
%     
% end
% %%
% q=[];
% for i=1:length(data)
%     q = [q; data{i}.joint_position];
% end
% plot(q)