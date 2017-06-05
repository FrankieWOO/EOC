function [ l_xx ] = get_hessian_fd( J, x )
%GET_HESSIAN_FD Summary of this function goes here
%   Detailed explanation goes here
    dimX = size(x,1);
    l_xx = zeros(dimX,dimX);
    delta = 1e-6;
    for i=1:dimX
	dx=zeros(dimX,1); dx(i)=delta;

	lxxp = J( x+dx);
	lxxm = J( x-dx);

	l_xx(:,i) = (lxxp-lxxm)/(2*delta);
	
    end

end

