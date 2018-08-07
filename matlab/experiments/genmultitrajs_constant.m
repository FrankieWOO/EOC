%%

if ~exist('target_list','var')
    genmultipletrajs_random_targets
end
%%

u2 = pi/6; % u2 fixed at pi/6
%target_list = zeros(num_targets, 1);
traj_list = cell(length(target_list),1);
% generate random targets
%q0 = init_position;
x0 = [init_position; 0; init_position; u2; 0; 0];

for iter = 1:length(target_list)
    traj = gentraj_constant(x0, target_list(iter), u2, 0.5);
    traj_list{iter} = traj;
    x0 = traj.xsim(:,end);
    %q0 = rd; % assume we can get to the target
end

%%
folderpath = 'trajs/constant/';
for k = 1 : length(target_list)
    filename = [folderpath,'command/','traj',num2str(k),'.csv'];
    savetraj_to_csv(filename, traj_list{k});
end

save([folderpath, 'constant.mat'])

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
