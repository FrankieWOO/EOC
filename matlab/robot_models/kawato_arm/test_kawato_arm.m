clear all

% set up simulation
dt = .02;              % control frequency
N  = 100;              % no. samples
t  = (0:N-1)*dt;       % sample times
x0 = [pi/3;pi/3;0;0];  % start state

% set up dynamics function
m  = model_kawato_arm; % model parameters
f = @(x, u) f_kawato_arm  ( x, u, m ); % dynamics function

% simulation parameter struct
p = [];
p.dt = dt;
p.solver = 'rk4';

% define feed forward commands
U = sin(5*rand(6,1)*t);

% simulate
X = simulate_feedforward ( x0, f, U, p );

% plot joint angles, velocities, commands
subplot(3,1,1),plot(X(1:2,:)'),ylabel('q (rad)')
subplot(3,1,2),plot(X(3:4,:)'),ylabel('dq/dt (rad/s)')
subplot(3,1,3),plot(U'),ylabel('u'),xlabel('t (s)')
