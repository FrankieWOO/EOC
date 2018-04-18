% Make one simulation step
% 
%     xn = simulate_step ( f, x, u, p )
% 
% in:
%    f  - dynamics (function handle)
%    x  - initial state
%    u  - command
%    p  - parameter struct containing:
%         p.solver - numberical solver, chosen from {'euler','rk4'}
%         p.dt     - time step
% 
% out: 
%    xn - next state
% 
function xn = simulate_step ( f, x, u, p )

switch p.solver
	case 'euler'
		dt=p.dt;
		xn = x + dt*f(x,u); % euler step
	case 'rk4'
		dt = p.dt;
        
		g1 = dt*f(x            ,u);
		g2 = dt*f(x+.5*g1,u);
		g3 = dt*f(x+.5*g2,u);
		g4 = dt*f(x+   g3,u);
		xn = x + (1/6)*(g1 + 2*g2 + 2*g3 + g4);
    case 'ode45'
        tspan = [0 p.dt];
        odefun = @(t,y)f(y,u);
        [~,y] = ode45(odefun,tspan,x);
        xn = y(end,:)';
	otherwise
		error('Unknown solver.')
end
end