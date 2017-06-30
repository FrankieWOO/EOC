% Simulate trajectory under closed-loop feedback control
%
%    [x,u] = simulate_feedback ( x0, f, pi, p )
%
% in:
%    x0 - initial state
%    f  - (discrete time) dynamics (function handle: x_{n+1}=f(x_n,u_n))
%    pi - policy (function handle representing the policy u=pi(x))
%    p  - parameter struct containing:
%         p.N      - number of steps to simulate
%         p.dt     - time step
% 
% out: 
%    x  - state trajectory 
%    u  - actions taken along the trajectory 
% 
function [x,u] = simulate_feedback ( x0, f, pi, p )

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

