target = 0;
T = 1;
dt = 0.02;
N = T/dt + 1;
t = 0:dt:T;
parm = [];
parm.has_gravity = true;
parm.m = 0.18;
parm.l = 0.085;
parm.b = 0.02;
pact = [];
pact.max_stiffness = 200;
pact.max_damping = 0.1;

robot = Pendulum1(parm,pact);
%%%% create task
ptask = [];
ptask.target = target;
ptask.model = robot;
ptask.w_t = 1000;
ptask.w_e = 1;
ptask.w_tf = 1000;
ptask.w_r = 1;
ptask.dt = dt;
task = pendulum1_reach(ptask);
x0 = [pi/4;0];
f = @(x,u)robot.dynamics(x,u);
j1 = @(x,u,t)task.j_fastreach(x,u,t);
j2 = @(x,u,t)task.j_fastreach_rege(x,u,t);
%% critical damped
psim.dt = 0.02;
psim.solver = 'rk4';

result0.k = 0;
result0.d = 0;
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
opt_param.umax(2) = 0;
opt_param.umin(2) = 0;

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
tsim = 0:0.001:T;
result1.usim = scale_controlSeq(result1.u,t(1:end-1),tsim(1:end-1));
psim.solver = 'rk4';
psim.dt = 0.001;
[result1.xsim] = simulate_feedforward(x0,f,result1.usim,psim);

result2.usim = scale_controlSeq(result2.u,t(1:end-1),tsim(1:end-1));
result2.xsim = simulate_feedforward(x0,f,result2.usim,psim);

result3.usim = scale_controlSeq(result3.u,t(1:end-1),tsim(1:end-1));
result3.xsim = simulate_feedforward(x0,f,result3.usim,psim);

result0.usim = scale_controlSeq(result0.u,t(1:end-1),tsim(1:end-1));
result0.xsim = simulate_feedforward(x0,f,result0.usim,psim);
%%
for i = 1:length(tsim)-1
    Prege0(i) = robot.power_rege(result0.xsim(:,i), result0.usim(:,i));
    tau0(i) = robot.torque_k(result0.xsim(:,i), result0.usim(:,i));
end
E0 = sum(Prege0)*psim.dt;
zeta0 = E0/(sum(tau0.*result0.xsim(2,1:end-1))*psim.dt);
for i = 1:length(tsim)-1
    Prege1(i) = robot.power_rege(result1.xsim(:,i), result1.usim(:,i));
    tau1(i) = robot.torque_k(result1.xsim(:,i), result1.usim(:,i));
end
E1 = sum(Prege1)*psim.dt;
zeta1 = E1/(sum(tau1.*result1.xsim(2,1:end-1))*psim.dt);
for i = 1:length(tsim)-1
    Prege2(i) = robot.power_rege(result2.xsim(:,i), result2.usim(:,i));
    tau2(i) = robot.torque_k(result2.xsim(:,i), result2.usim(:,i));
end
E2 = sum(Prege2)*0.001;
Ek2 = sum(tau2.*result2.xsim(2,1:end-1))*0.001;
zeta2 = E2/Ek2;
for i = 1:length(tsim)-1
    Prege3(i) = robot.power_rege(result3.xsim(:,i), result3.usim(:,i));
    tau3(i) = robot.torque_k(result3.xsim(:,i), result3.usim(:,i));
end
E3 = sum(Prege3)*psim.dt;
zeta3 = E3/(sum(tau3.*result3.xsim(2,1:end-1))*psim.dt);
E4 = 0; zeta4=0;
%%
figure
subplot(3,2,1)
plot(t, result0.x(1,:),'--')
hold on
%plot(t, ones(1,N)*target)
plot(t, result1.x(1,:),'-','LineWidth',3,'Color', [1 0.6 0.6])
plot(t, result1.x(1,:),'k-','LineWidth',1)

plot(t, result2.x(1,:),'b-.','LineWidth',1)
plot(t, result3.x(1,:),'-','Color', [1 0.6 0.6], 'LineWidth', 1.5)
hold off
legend('C.D.','dynamic', 'hybrid','regenerative','fixed damping')
subplot(3,2,2)
hold on
plot(t(1:end-1), result0.u(1,:),'--')
plot(t(1:end-1), result1.u(1,:),'-','LineWidth',3,'Color', [1 0.6 0.6])

plot(t(1:end-1), result1.u(1,:),'k-','LineWidth',1)
plot(t(1:end-1), result2.u(1,:),'b-.','LineWidth',1)
plot(t(1:end-1), result3.u(1,:),'-','Color', [1 0.6 0.6], 'LineWidth', 1.5)

hold off


subplot(3,2,3)
hold on
plot(t(1:end-1), result0.u(2,:)*robot.actuator.max_stiffness,'--')
plot(t(1:end-1), result1.u(2,:)*robot.actuator.max_stiffness,'-','LineWidth',3,'Color', [1 0.6 0.6])

plot(t(1:end-1), result1.u(2,:)*robot.actuator.max_stiffness,'k-','LineWidth',1)
plot(t(1:end-1), result2.u(2,:)*robot.actuator.max_stiffness,'b-.','LineWidth',1)
plot(t(1:end-1), result3.u(2,:)*robot.actuator.max_stiffness,'-','Color', [1 0.6 0.6], 'LineWidth', 1.5)
hold off

subplot(3,2,4)
hold on
plot(t(1:end-1), result0.u(3,:)*robot.actuator.max_damping,'--')
plot(t(1:end-1), result1.u(3,:)*robot.actuator.max_damping,'-','LineWidth',3,'Color', [1 0.6 0.6])

plot(t(1:end-1), result1.u(3,:)*robot.actuator.max_damping,'k-','LineWidth',1)
plot(t(1:end-1), result2.u(3,:)*robot.actuator.max_damping,'b-.')
plot(t(1:end-1), result3.u(3,:)*robot.actuator.max_damping,'-','Color', [1 0.6 0.6], 'LineWidth', 1.5)
hold off

subplot(3,2,5)
c = categorical({'C.D.','dynamic','regenerative','hybrid','fixed damp'});
Y = [E0/zeta0,E1/zeta1,E2/zeta2,E1/zeta1,E3/zeta3];
Erege = [E0,E4,E2,E1,E3];
hold on
bar(c, Y, 'FaceColor',[0.5 0 .5],'EdgeColor',[0.9 0 .9],'LineWidth',1.5)
bar(c, Erege, 'FaceColor',[0 .5 .5],'EdgeColor',[0 .9 .9],'LineWidth',1.5)
hold off