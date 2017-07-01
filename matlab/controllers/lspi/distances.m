% function D = distances(A,B)
% 
% [d ,m] = size(A);
% [d2,n] = size(B);
% 
% if (d~=d2) 
%    error('bad dims')
% end
% 
% a2 = sum(A.^2);
% b2 = sum(B.^2);
% 
% D = repmat(a2',1,n) + repmat(b2,m,1) - 2*A'*B;
% end

function D = distances(A,B)

[d,m] = size(A);
[d2,n] = size(B);

if (d~=d2) 
   error 'bad dims' 
end

a2 = sum(A.^2,1);
b2 = sum(B.^2,1);

D = repmat(a2',1,n) + repmat(b2,m,1) - 2*A'*B;