% Function for solving (infinite horizon) optimal control problems with the Least Squares Policy Iteration method.
%
%    [ pi, Q, model ] = lspi ( D, pi0, model, argmaxuQ, gamma, p )
%
% in:
%     D                 - data struct, containing samples of
%      .Y               - state measurements
%      .U               - commands
%      .R               - cost (reward)
%      .Yn              - next state measurements
%     pi0               - initial policy (function handle) 
%     phi               - basis functions (function handle) 
%     argmaxuQ          - function for finding the argmax_u (Q) (i.e., converting theta to a policy function (function handle) 
%     gamma             - discount factor
%     p                 - parameter struct (optionally) containing:
%      .iter_max        - threshold on number of iterations (exit if exceeded)
%      .dtheta_converge - threshold on relative improvement in cost (exit if improvement less than this value)
%      .online_printing - print: {0:never, 1:every iteration, 2:final iteration only}
%                         (any fields that are unspecified will be set by default values).
%
% out:
%     pi                - learnt policy (function handle)
%     Q                 - Q-function (function handle)
%     model             - learnt Q-function model
%
function [ pi, Q, model ] = lspi ( D, pi0, model, argmaxuQ, gamma, p )

% unpack parameter struct
if isfield(p,'evalobjective'  ),evalobjective   = p.evalobjective  ;else,evalobjective   = 'lsfp';end
if isfield(p,'dtheta_converge'),dtheta_converge = p.dtheta_converge;else,dtheta_converge =  1e-9 ;end
if isfield(p,'iter_max'       ),iter_max        = p.iter_max       ;else,iter_max        =  100  ;end
if isfield(p,'online_printing'),online_printing = p.online_printing;else,online_printing =  1    ;end
if isfield(p,'p_lstd0q'       ),p_lstd0q        = p.p_lstd0q       ;else,p_lstd0q        =  []   ;end

% unpack data
Y = D.Y;
U = D.U; 
R = D.R; 
Yn= D.Yn;
[dimY N] = size(Y);             % get y dimensionality, no. data points
 dimU    = size(U,1);           % get u dimensionality

% initialise matrices
Un = zeros(dimU,N);

% initialise policy
pi = pi0;

% main loop
for iter=1:iter_max

	% sample actions from current policy
	Un = pi(Yn);

	% estimate Q
	model = lstd0q ( Y, U, R', Yn, Un, model, gamma, p_lstd0q );
	Q = @(y,u)-predict_linear([y;u],model);

    % find the policy by taking the argmax
	pi = argmaxuQ(model.w); 

	% compute root mean squared temporal difference error
	Qp = Q(Y ,U );
	Qpn= Q(Yn,Un);
	rmstde = fn_error_rmstde(Qp,Qpn,R,gamma);

	% print stuff out
    if online_printing==1,fprintf('Iteration = %d, RMSTDE = %e, Qp = %f +/- %f (mean +/- s.d.)\n',iter,rmstde,mean(Qp),std(Qp));end

	% check for convergence
	if iter>1 
		% find change in theta
		dtheta = norm(model.w(:)-modelp.w(:));
		if dtheta<dtheta_converge
			fprintf('Minimum parameter change threshold reached. '); break; % parameter change too small: EXIT
		end
	end
	modelp = model;
end
if iter==iter_max
	fprintf('Max. number of iterations reached. ');
end
% print final result if necessary
if online_printing==2
    fprintf('Iterations = %d; ', iter);
end
fprintf('\n');

