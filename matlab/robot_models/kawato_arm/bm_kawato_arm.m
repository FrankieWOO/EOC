% Function for computing the muscle damping for the Kawato arm model.
%
%  in:
%      u      - muscle activations
%      model  - model struct
%
%  out:
%      bm     - muscle damping
%
function bm = bm_kawato_arm(u,model) %#eml

b0 = model.b0;  % muscle damping parameters
gb = model.gb;  %
bm = b0 + gb.*u;

%> @file get_muscle_damping_kawato_arm.m
%> @author Matthew Howard (MH), matthew.howard@ed.ac.uk
%> @date 11/07/11 17:50:47
