% Function for calculating the Jacobian of a cost function using finite differences.
% 
% in:
%    l       - function handle for calculating cost
%    x, u, t - state, command and time at which Jacobian should be calculated
%
% out:
%    l_x,l_u - Jacobian w.r.t. state and action.
%
function [l_x,l_u] = J_cost_fd ( l, x, u, t )

dimX = size(x,1);
dimU = size(u,1);

delta=1e-6;

l_x  = zeros(dimX,1);
for i=1:dimX
	dx=zeros(dimX,1); dx(i)=delta;

	lxp = l( x+dx, u, t );
	lxm = l( x-dx, u, t );

	l_x(i) = (lxp-lxm)/(2*delta);
end

l_u  = zeros(dimU,1);
for i=1:dimU
	du=zeros(dimU,1); du(i)=delta;

	lup = l( x, u+du, t );
	lum = l( x, u-du, t );

	l_u(i) = (lup-lum)/(2*delta);
end

end