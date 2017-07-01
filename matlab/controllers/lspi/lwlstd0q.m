% Function for estimating the Q-function of a policy through Locally Weighted Least Squares Temporal Difference learning (i.e., TD(0)).
%
%    model = lwlstd0q ( Y, U, R, Yn, Un, model, gamma, p )
%
% in:
%     Y                 - state samples
%     U                 - action samples
%     R                 - reward samples
%     Yn                - next state samples
%     Un                - next action samples
%     model             - Q-function model struct, containing
%          .phi         - local feature function
%          .W           - local weighting function
%     gamma             - discount factor
%     p                 - parameter struct (optionally) containing:
%      .regularisation  - regularisation method {'eigenvalue': eigenvalue decomposition, 'pinv': Matlab's inbuilt pinv, 'ridge': ridge-regression}
%       (any fields that are unspecified will be set by default values).
%
% out:
%     model             - updated Q-function model, containing
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
function model = lwlstd0q ( Y, U, R, Yn, Un, model, gamma, p )

model = lwlstd0 ( [Y;U], R, [Yn;Un], model, gamma, p );

