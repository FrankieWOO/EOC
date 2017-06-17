%curPath = strsplit(pwd,{'\','/'});
%curPath{1,end+1} = 'external/genpath_exclude/';
%addpath(strjoin(curPath,'/'));
%addpath(genpath_exclude(pwd,{'.git','.svn','ros_package'}));
%addpath(fullfile(pwd,'ros_package','matlab_gen','msggen'));
%addpath('~/lwpr/matlab/');
addpath('/home/fan/catkin_ws/src/matlab_gen/msggen');
curPath = pwd;
addpath(genpath(pwd));
clear curPath