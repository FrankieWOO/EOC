
if ~exist('pubcmd','var')
    init_ros
    
end

%% define reset traj
ureset = repmat([0;0;0],1,100);
y=exetraj(ureset);

%% define
%tj_hybrid = load('g20/tj_hybrid.mat');
%u = tj_hybrid.result3.u;

u0 = [pi/4;0;0.5];
u = repmat(u0,1,100);
y = exetraj_plot(u);

