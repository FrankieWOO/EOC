% Function for estimating the V-function of a policy through Least Squares Temporal Difference learning (i.e., TD(0)).
%
%    model = lstd0 ( X, R, Xn, model, gamma )
%
% in:
%     X                 - state samples
%     R                 - reward samples
%     Xn                - next state samples
%     model             - V-function model struct, containing
%          .phi         - feature function
%     gamma             - discount factor
%
% out:
%     model             - updated V-function model, containing
%          .phi         - feature function
%          .w           - model weights
%
% NOTE: 
%    1) This finds the Least Squares Fixed Point
%       approximation to the value function (see Lagoudakis,
%       2003). The latter appears to be the most commonly used
%       approximation in LSTD learning.
%    2) This is equivalent to calling learn_lstdl.m with
%       lambda=0, but more efficient (no loops) and without
%       the requirement that data is presented as
%       trajectories.
%
function model = lstd0 ( X, R, Xn, model, gamma, p )

% get no. data points
N = size(X,2);

% evaluate basis functions on all data
Phi  = model.phi(X)';

% evaluate next-step basis functions
Phin = model.phi(Xn)';

% compute LSTD(0) solution
model.w = wlstd0 ( Phi, Phin, R, ones(1,N), gamma, p );

