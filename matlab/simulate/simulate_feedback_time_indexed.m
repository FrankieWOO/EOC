% Simulate trajectory using Euler or rk4 under closed-loop, time-indexed policy control, in continuous time
% 
% in:
%    x0 - initial state
%    f  - (continuous time) dynamics (function handle: xdot=f(x,u))
%    pi - time-indexed policies (function handle: u=pi(x,n))
%    p  - parameter struct containing:
%     .N  - number of steps to simulate
%     .dt - time step
% 
% 
% out: 
%     x,u     - state,action trajectory 
% 
function [x,u] = simulate_feedback_time_indexed ( x0, f, pi, p )

%dt= p.dt;
N = p.N;
x = nan(size(x0,1),N); x(:,1)=x0; % initialise x
u = nan(size(pi(x0,1),1),N-1);      % initialise u
% simulate
for n=1:N-1
	u(:,n  ) = pi(x(:,n),n);                 % get next action 
	%x(:,n+1) = x(:,n) + dt*f(x(:,n),u(:,n)); % euler step
    x(:,n+1) = simulate_step ( f, x(:,n), u(:,n), p );
end

end

