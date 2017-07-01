% Function for computing weighted Least Squares Temporal Difference learning estimators.
%
%    theta = wlstd0 ( Phi, R, Phin, W, gamma )
%
% in:
%     Phi               - feature matrix (NxM)
%     R                 - reward sample vector (Nx1)
%     Phin              - next state/action feature matrix (NxM)
%     W                 - weight vector (1xN)
%     gamma             - discount factor
%     p                 - parameter struct (optionally) containing:
%      .regularisation  - regularisation method {'svd': singular value decomposition, 'eigenvalue': eigenvalue decomposition, 'pinv': Matlab's inbuilt pinv, 'ridge': ridge-regression, default: none}
%       (any fields that are unspecified will be set by default values).
%
% out:
%     theta             - parameter estimate (Mx1)
%
% NOTE: 
%    1) This finds the Least Squares Fixed Point
%       approximation to the value function (see Lagoudakis,
%       2003). The latter appears to be the most commonly used
%       approximation in LSTD learning.
%    2) Set W = ones(1,N) for ordinary (non-weighted) LSTD(0).
%
function theta = wlstd0 ( Phi, Phin, R, W, gamma, p )

% unpack parameter struct
if isfield(p,'regularisation'),regularisation = p.regularisation;else regularisation = 'none';end

dimPhi = size(Phi,2);

% compute temporal difference
dPhi = Phi-gamma*Phin;

% compute weighted Phi
WPhi=repmat(W,dimPhi,1)'.*Phi;

% compute statistics A,b (vectorised calculation)
A=WPhi'*dPhi;
b=WPhi'*R;

% compute parameter estimate
switch regularisation
case 'svd'
	% SVD inversion
	[U,S,V] = svd(A); 
	sv = diag(S);
	i = find(sv>p.min_sv); 
	sv(i) = sv(i).^-1; pinvA1 = V(:,i)*diag(sv(i))*U(:,i)'; theta=pinvA1*b;
	%% --- add nullspace term ---
	%ind = find(ev<1e-6); ev(ind) = 0; 
	%theta = theta + V(:,ind)*((2*rand-1)*ones(size(V(:,ind),2),1));
	%fprintf(1,'Removed singular values %i/%i\n',length(ind),length(ev))
	%% ---                    ---
case 'eigenvalue'
	% eigen-decomposition inversion
	[V,D] = eig(A); ev = diag(D); ind = find(ev>p.min_ev); V1=V(:,ind); pinvA1 = V1*diag(ev(ind).^-1)*V1'; theta=pinvA1*b;
case 'pinv'
	% pseudoinverse
	theta=pinv(A)*b;
case 'ridge'
	% inversion with tikhonov regurlarisation
	theta=(A+p.lambda*eye(size(A)))\b;
case 'none'
	% Gaussian elimination
	theta=A\b;
otherwise
	warning('Unknown regularisation scheme, defaulting to ''none''.')
	theta=A\b;
end

% The below computes the error in the inversion
%sum((A*theta-b).^2,1)

