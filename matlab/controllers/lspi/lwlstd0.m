% Function for estimating the V-function of a policy through Locally Weighted Least Squares Temporal Difference learning (i.e., TD(0)).
%
%    model = lwlstd0 ( Y, R, Yn, model, gamma, p )
%
% in:
%     Y                 - state samples
%     R                 - reward samples
%     Yn                - next state samples
%     model             - V-function model struct, containing
%          .phi         - local feature function
%          .W           - local weighting function
%     gamma             - discount factor
%     p                 - parameter struct (optionally) containing:
%      .regularisation  - regularisation method {'svd': singular value decomposition, 'eigenvalue': eigenvalue decomposition, 'pinv': Matlab's inbuilt pinv, 'ridge': ridge-regression, default: none}
%       (any fields that are unspecified will be set by default values).
%
% out:
%     model             - updated V-function model, containing
%          .phi         - local feature function
%          .W           - local weighting function
%          .w           - model weights
%
% NOTE: 
%    1) This finds the Least Squares Fixed Point
%       approximation to the value function (see Lagoudakis,
%       2003). The latter appears to be the most commonly used
%       approximation in LSTD learning.
%
function model = lwlstd0 ( Y, R, Yn, model, gamma, p )

% find weights
W = model.W(Y);
Nc = size(W,1);   % get no. centres

% evaluate basis functions on all data
Phi  = model.phi(Y)';

% evaluate next-step basis functions
Phin = model.phi(Yn)';

% estimate parameters for each local model
for nc=1:Nc
	model.w(:,:,nc) = wlstd0 ( Phi, Phin, R, W(nc,:), gamma, p );
end

