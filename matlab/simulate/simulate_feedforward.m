% Simulate trajectory under open-loop, feed forward control, in continuous time
% 
%     x = simulate_feedforward ( x0, f, u, dt )
% 
% in:
%    x0 - initial state
%    f  - (continuous time) dynamics (function handle: xdot=f(x,u))
%    u  - command sequence
%    p  - parameter struct containing:
%         p.solver - numerical solver (see simulation_step.m)
%         p.dt     - time step
% 
% out: 
%    x        - state trajectory 
% 
function x = simulate_feedforward ( x0, f, u, p )

N = size(u,2)+1; % N of time index
x = nan(size(x0,1),N); x(:,1)=x0; % initialise x
for n=1:N-1
	x(:,n+1) = simulate_step ( f, x(:,n), u(:,n), p );
end

end


