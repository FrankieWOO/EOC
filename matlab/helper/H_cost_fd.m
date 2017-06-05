% Function for calculating the Hessian of a cost function using finite differences on the cost Jacobian.
% 
% in:
%    J       - function handle for calculating cost Jacobian
%    x, u, t - state, command and time at which Hessian should be calculated
%
% out:
%    l_xx,l_uu,l_ux - Hessian w.r.t. state, action and cross term.
%
function [l_xx,l_uu,l_ux] = H_cost_fd ( J, x, u, t )

dimX = size(x,1);
dimU = size(u,1);

delta=1e-6;

l_xx = zeros(dimX,dimX);
l_ux = zeros(dimU,dimX);
for i=1:dimX
	dx=zeros(dimX,1); dx(i)=delta;

	[lxxp,luxp] = J( x+dx, u, t );
	[lxxm,luxm] = J( x-dx, u, t );

	l_xx(:,i) = (lxxp-lxxm)/(2*delta);
	l_ux(:,i) = (luxp-luxm)/(2*delta);
end

l_uu = zeros(dimU,dimU);
for i=1:dimU
	du=zeros(dimU,1); du(i)=delta;

	[dummy,luup] = J( x, u+du, t );
	[dummy,luum] = J( x, u-du, t );

	l_uu(:,i) = (luup-luum)/(2*delta);
end

end