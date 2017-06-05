% Simulate trajectory using Euler integration under closed-loop, time-indexed policy control, in continuous time
% 
% in:
%    x0 - initial state
%    f  - (continuous time) dynamics (function handle: xdot=f(x,u))
%    pi - time-indexed policies (function handle: u=pi(x,n))
%    p  - parameter struct containing:
%     .N  - number of steps to simulate
% 
% 
% out: 
%     x,u     - state,action trajectory 
% 
function [x,u] = simulate_feedback_time_indexed_d ( x0, f, pi, p )

N = p.N;
x = nan(size(x0,1),N); x(:,1)=x0; % initialise x
u = nan(size(pi(x0,1),1),N-1);      % initialise u
% simulate
for n=1:N-1
	u(:,n  ) = pi(x(:,n),n);                 % get next action 
	x(:,n+1) = f(x(:,n),u(:,n)); % euler step
end

