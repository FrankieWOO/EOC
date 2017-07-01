% Calculate acceleration for Kawato arm model given current joint angles, velocities and muscle activations.
%
%  in:
%     q     - joint angles
%     qdot  - joint velocities
%     u     - muscle activations
%     model - model struct
%
% out:  
%      qddot - acceleration
% 
function qddot = qddot_kawato_arm ( q, qdot, u, model )

tau   = tau_kawato_arm ( q, qdot, u, model );
qddot = qddot_planar_2_link_arm ( q, qdot, tau, model );


