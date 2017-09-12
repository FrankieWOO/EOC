function [x] = simulate_fixu(f,x0,u,dt,N)
x = zeros(size(x0,1),N+1);
x(:,1) = x0;
p.solver = 'rk4';
p.dt = dt;
for i = 1:N
    x(:,i+1) = simulate_step(f,x(:,i),u,p);
end
end