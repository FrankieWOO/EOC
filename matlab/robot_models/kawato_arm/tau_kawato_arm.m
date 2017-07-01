% Calculate torque for Kawato arm model given current joint angles, velocities and muscle activations.
% 
%  in:
%      q     - joint angles
%      qdot  - joint velocities
%      u     - muscle activations
%      model - model struct
%
%  out:
%      tau   - torque
%
function tau = tau_kawato_arm ( q, qdot, u, model )

A   = model.A ; % moment arms
T   = Tm_kawato_arm ( q, qdot, u, model );
tau = A'*T; % calculate torque

