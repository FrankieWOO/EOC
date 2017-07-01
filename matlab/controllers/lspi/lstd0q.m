% Function for estimating the Q-function of a policy through Least Squares Temporal Difference learning (i.e., TD(0)).
%
%    model = lstd0q ( Y, U, R, Yn, Un, model, gamma )
%
% in:
%     Y                 - state samples
%     U                 - action samples
%     R                 - reward samples
%     Yn                - next state samples
%     Un                - next action samples
%     model             - Q-function model struct, containing
%          .phi         - feature function
%     gamma             - discount factor
%
% out:
%     model          - updated Q-function model, containing
%          .phi      - feature function
%          .w        - model weights
%
% NOTE: 
%    1) This finds the Least Squares Fixed Point
%       approximation to the value function (see Lagoudakis,
%       2003). The latter appears to be the most commonly used
%       approximation in LSTD learning.
%
function model = lstd0q ( Y, U, R, Yn, Un, model, gamma, p )

Z  = [Y ;U ];
Zn = [Yn;Un];

model = lstd0 ( Z, R, Zn, model, gamma, p );

