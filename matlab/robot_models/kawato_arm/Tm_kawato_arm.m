% Calculate muscle tension for Kawato arm model given current joint angles, velocities and muscle activations.
% 
%  in:
%      q     - joint angles
%      qdot  - joint velocities
%      u     - muscle activations
%      model - model struct
%
%  out:
%      T     - muscle tensions
%
function T = T_kawato_arm ( q, qdot, u, model )

N = size(q,2);

A    = model.A ; % moment arms
gr   = model.gr; % constant
lm_l0= model.lm_l0;

for n=1:N
	km   = km_kawato_arm(u(:,n),model); 
	bm   = bm_kawato_arm(u(:,n),model);

	ldot = -A*qdot(:,n);

	T(:,n) = km.*(lm_l0 - A*q(:,n) + gr.*u(:,n)) + bm.*ldot;
end


