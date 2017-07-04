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
%      .umax
%      .umin
% out:
%     u  - action
%
function u = pi_ilqr ( x, t, p )

    u = p.un(:,t) + p.Ln(:,:,t)*(x - p.xn(:,t));

    if isfield(p,'umax') && isfield(p, 'umin')
        u = min(max(u,p.umin),p.umax);
    end

end