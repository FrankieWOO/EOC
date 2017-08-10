
target = pi/3;
T = 1;
dt = 0.02;
N = T/dt + 1;
t = 0:dt:T;
parm = [];
parm.has_gravity = false;
pact = [];
pact.max_stiffness = 200;
pact.max_damping = 50;

robot = Pendulum1(parm,pact);
%%%% create task
ptask = [];
ptask.target = target;
ptask.model = robot;
ptask.w_t = 1000;
ptask.w_e = 1;
ptask.w_tf = 1000;
ptask.w_r = 0.1;
ptask.dt = dt;
task = pendulum1_reach(ptask);
x0 = [0;0];
f = @(x,u)robot.dynamics(x,u);
j1 = @(x,u,t)task.j_fastreach(x,u,t);

%% critical damped
psim.dt = 0.02;
psim.solver = 'rk4';

result0.k = 100;
result0.d = 2*sqrt(result0.k);
result0.u = repmat( [target; result0.k/pact.max_stiffness; result0.d/pact.max_damping], 1, N-1 );
result0.x = simulate_feedforward(x0,f,result0.u,psim);
%% 1 dynamic braking & hybrid mode
opt_param = [];
opt_param.umax = robot.umax; opt_param.umax(3) = 1;
opt_param.umin = robot.umin; 
opt_param.lambda_init = 0.5;
opt_param.lambda_max  = 1e10;
opt_param.iter_max = 300;
opt_param.online_plotting = 0;
opt_param.online_printing = 1;
opt_param.dcost_converge = 1e-9;
opt_param.solver = 'rk4';
opt_param.target = target;

opt_param.T = T;

opt_param.umax(1) = target;
opt_param.umin(1) = target;

% u0 can be full command sequence or just initial point
%u0 = result0.u;
u0 = [target; 0; 0];

%result1 = ILQRController.ilqr(f, j1, dt, N, x0, u0, opt_param);
%% 2 pure regenerative braking
u3_max = robot.actuator.u_max_regedamp;
opt_param2 = opt_param;
opt_param2.umax(3) = u3_max;
result2 = ILQRController.ilqr(f, j1, dt, N, x0, u0, opt_param2);

result1 = ILQRController.ilqr(f, j1, dt, N, x0, u0, opt_param);

%% 3 fixed max rege power damping
opt_param3 = opt_param;
opt_param3.umax(3) = u3_max;
opt_param3.umin(3) = u3_max;
result3 = ILQRController.ilqr(f, j1, dt, N, x0, u0, opt_param3);
%%
for i = 1:N-1
    Prege0(i) = robot.power_rege(result0.x(:,i), result0.u(:,i));
end
E0 = sum(Prege0);
for i = 1:N-1
    Prege1(i) = robot.power_rege(result1.x(:,i), result1.u(:,i));
end
E1 = sum(Prege1);
for i = 1:N-1
    Prege2(i) = robot.power_rege(result2.x(:,i), result2.u(:,i));
end
E2 = sum(Prege2);
for i = 1:N-1
    Prege3(i) = robot.power_rege(result3.x(:,i), result3.u(:,i));
end
E3 = sum(Prege3);

E4 = 0;
%%
figure
subplot(3,2,1)
plot(t, result0.x(1,:),'--')
hold on
%plot(t, ones(1,N)*target)
plot(t, result1.x(1,:),'r-','LineWidth',1)
plot(t, result2.x(1,:),'b-.','LineWidth',1)
plot(t, result3.x(1,:),'-','Color', [1 0.6 0.6], 'LineWidth', 1.5)
hold off

subplot(3,2,2)
hold on
plot(t(1:end-1), result0.u(1,:),'--')
plot(t(1:end-1), result1.u(1,:),'r-','LineWidth',1)
plot(t(1:end-1), result2.u(1,:),'b-.','LineWidth',1)
plot(t(1:end-1), result3.u(1,:),'-','Color', [1 0.6 0.6], 'LineWidth', 1.5)

hold off


subplot(3,2,3)
hold on
plot(t(1:end-1), result0.u(2,:)*robot.actuator.max_stiffness,'--')
plot(t(1:end-1), result1.u(2,:)*robot.actuator.max_stiffness,'r-','LineWidth',1)
plot(t(1:end-1), result2.u(2,:)*robot.actuator.max_stiffness,'b-.','LineWidth',1)
plot(t(1:end-1), result3.u(2,:)*robot.actuator.max_stiffness,'-','Color', [1 0.6 0.6], 'LineWidth', 1.5)
hold off

subplot(3,2,4)
hold on
plot(t(1:end-1), result0.u(3,:)*robot.actuator.max_damping,'--')
plot(t(1:end-1), result1.u(3,:)*robot.actuator.max_damping,'r-','LineWidth',1)
plot(t(1:end-1), result2.u(3,:)*robot.actuator.max_damping,'b-.')
plot(t(1:end-1), result3.u(3,:)*robot.actuator.max_damping,'-','Color', [1 0.6 0.6], 'LineWidth', 1.5)

subplot(3,2,5)
c = categorical({'C.D.','dynamic','regenerative','hybrid','fixed damp'});
bar(c, [E0,E4,E2,E1,E3], 'FaceColor',[0 .5 .5],'EdgeColor',[0 .9 .9],'LineWidth',1.5)