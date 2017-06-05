% Finite horizon linear policy
%
%  u = pi_linear ( x, n, L )
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
function u = pi_linear_fh ( x, n, L )

u=-L(:,:,n)*x;

