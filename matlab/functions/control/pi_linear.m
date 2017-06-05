% Linear policy
%
%  u = fn_policy_linear ( x, L )
%
% Implements a policy of the form 
%
%  u = -L x
%
% in:
%     x - state
%     L - parameters (gains)
%
% out:
%     u  - action
%
function u = pi_linear ( x, L )

u=-L*x;

