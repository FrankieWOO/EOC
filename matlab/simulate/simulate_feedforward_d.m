% Simulate trajectory under open-loop, feed forward control, in continuous time
% 
%     x = simulate_feedforward ( x0, f, u, dt )
% 
% in:
%    x0 - initial state
%    f  - (discrete time) dynamics (function handle: x_{n+1}=f(x_n,u_n))
%    u  - command sequence
% 
% out: 
%    x        - state trajectory 
% 
function x = simulate_feedforward_d ( x0, f, u )

N = size(u,2)+1;
x = nan(size(x0,1),N); x(:,1)=x0; % initialise x
for n=1:N-1
	x(:,n+1) = f(x(:,n),u(:,n));
end


