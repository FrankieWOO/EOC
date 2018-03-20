function []=draw_ellipse(M,C,w,kernel)
% function draw ellipse draws the ellipse corresponding to the
% eigenvalues of M at the location c.

[V,E] = eig(M);

E = E;
d1 = E(1,1);
d2 = E(2,2);

steps = 50;
switch kernel
case 'Gaussian'
	start = sqrt(-2*log(w)/d1);
case 'BiSquare'
	start = sqrt(4*(1-sqrt(w))/d1);
end


for i=0:steps,
	Xp(i+1,1) = -start + i*(2*start)/steps;
	switch kernel
	case 'Gaussian'
		arg = (-2*log(w)-Xp(i+1,1)^2*d1)/d2;
	case 'BiSquare'
		arg = (4*(1-sqrt(w))-Xp(i+1,1)^2*d1)/d2;
	end
	if (arg < 0), 
		arg = 0; 
	end; % should be numerical error
	Yp(i+1,1) = sqrt(arg);
end;

for i=1:steps+1;
	Xp(steps+1+i,1) = Xp(steps+1-i+1,1);
	Yp(steps+1+i,1) = -Yp(steps+1-i+1,1);
end;

% transform the rf

M = [Xp,Yp]*V(1:2,1:2)';

Xp = M(:,1) + C(1);
Yp = M(:,2) + C(2);

plot(C(1),C(2),'ro',Xp,Yp,'c');
end

