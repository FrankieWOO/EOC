function [X,Y,Z]=makesurf(data,nx)
% %% auxiliary functions for plot 3D surface
% [X,Y,Z]=makesurf(data,nx) converts the 3D data file data into
% three matices as need by surf(). nx tells how long the row of the
% output matrices are

[m,n]=size(data);

n=0;
for i=1:nx:m,
	n = n+1;
	X(:,n) = data(i:i+nx-1,1);
	Y(:,n) = data(i:i+nx-1,2);
	Z(:,n) = data(i:i+nx-1,3);
end
end

