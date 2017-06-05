% LWLSPI policy
%
%  u = fn_policy_lwlspi ( x, L )
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
function u = fn_policy_lwlspi ( x, Lx, Lu, L1, W, p )

[dimX N] = size(x);  % get no. data points, dimensionality of X
dimU = size(Lu,2);   % get dimensionality of U

if isfield(p,'umax' ),umax  = p.umax ;else,umax  =  inf(dimU,1);end
if isfield(p,'umin' ),umin  = p.umin ;else,umin  = -inf(dimU,1);end

% find weights
W = W(x);
Nc = size(W,1);   % get no. local models

% compute action
u    = zeros(dimU,N);
wLu  = zeros(dimU,dimU,N);
wLx  = zeros(dimU,dimX,N);
wL1  = zeros(dimU,   1,N);
for n=1:N
	for nc=1:Nc
	wLu(:,:,n) = wLu(:,:,n) + W(nc,n)*Lu(:,:,nc);
	wLx(:,:,n) = wLx(:,:,n) + W(nc,n)*Lx(:,:,nc);
	wL1(:,:,n) = wL1(:,:,n) + W(nc,n)*L1(:,:,nc);
	end
	u(:,n) = -(wLu(:,:,n))\([wLx(:,:,n),wL1(:,:,n)]*[x(:,n);1]);
end

% enforce command limits
for m=1:dimU
	i=find(u(m,:)>umax(m)); u(m,i)=umax(m);
	i=find(u(m,:)<umin(m)); u(m,i)=umin(m);
end
