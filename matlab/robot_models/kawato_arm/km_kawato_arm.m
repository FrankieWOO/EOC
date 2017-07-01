% Function for computing the muscle stiffnesses for the Kawato arm model.
%
%  in:
%      u      - muscle activations
%      model  - model structure
%
%  out:
%      km     - muscle stiffnesses
%
function km = km_kawato_arm(u,model) %#eml

k0 = model.k0;  % muscle stiffness parameters
gk = model.gk;  %
km = k0 + gk.*u;

%> @file get_muscle_stiffness_kawato_arm.m
%> @author Matthew Howard (MH), matthew.howard@ed.ac.uk
%> @date 11/07/11 17:50:47
