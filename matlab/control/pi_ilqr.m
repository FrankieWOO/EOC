% ILQR/G policy
%
% u = u0 + L (x-x0).
%
% in:
%     x    - state
%     t    - time step index
%     p    - paramter struct, containing
%      .un - nominal command
%      .xn - nominal state
%      .Ln - gains
%
% out:
%     u  - action
%
function u = pi_ilqr ( x, t, p )

un = p.un;
xn = p.xn;
Ln = p.Ln;

u = un(:,t) + Ln(:,:,t)*(x-xn(:,t));
end