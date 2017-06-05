% Function for calculating the Jacobian of a function with finite central differences.
%
%     J = get_jacobian_fd ( f, x )
%
% Computes the Jacobian of the function f with finite central differences.
% 
% in: 
%     f     - function handle to the function
%     x     - point in input space where Jacobian should be evaluated.
%
% out:
%     J - Jacobian
%
% Note: There is also a MEX version of this function - see get_jacobian_fd.c
%
function J = get_jacobian_fd ( f, x )

delta=1e-6;
for i=1:length(x)
	dx = zeros(size(x)); dx(i) = delta;
	yp = f(x+dx);
	ym = f(x-dx);
	J(:,i) = ((yp(:) - ym(:))/(2*delta))';
end

% (below is Doxygen documentation)
%> \file get_jacobian_fd.m
%> \author Matthew Howard (MH), matthew.howard@ed.ac.uk
%> \date 30/06/11 14:11:48
%>
%> \brief Function for calculating the Jacobian of a function with finite central differences.
%> \sa get_jacobian_fd.c

