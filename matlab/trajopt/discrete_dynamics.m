function xnew = discrete_dynamics(f, x, u)
psim.dt = 0.02;
psim.solver='rk4';
xnew = simulate_step(f, x, u, psim);
end

