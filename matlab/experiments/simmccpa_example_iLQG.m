%function simmccpa_example_iLQG
%param_act.ratio_load = 0;
param_act.ratio_load = 1;
param_act.gear_d = 40;
%param_act.Kd = 0.0212;
%param_act.K1 = 1;
%param_act.K2 = 1;
%param_act.R1 = 0.5;
%param_act.R2 = 0.5;
%param_act.Ks = 500;
%param_act.J1 = 0.001;
%param_act.J2 = 0.001;
robot_param=[];
robot_param.inertia_l = 0.0015;
robot_param.Df = 0.004;
robot_model = Mccpvd1dofModel(robot_param, param_act);

target = pi/4;
T = 2;
dt = 0.02;
N = T/dt + 1;
t = 0:dt:T;
%alpha = 0.7;
position0 = 0;
x0 = zeros(6,1); 
x0(1) = 0;
x0(3) = 0; % initial motor1
x0(4) = 0; % initial motor2

% task_param = [];
% task_param.target = target;
% task_param.T = T;
% task_param.w = w;
% task_param.rege_ratio = alpha;
% task_param.position0 =  position0;
% task_param.x0 = x0; 
% %define the initial spring pretension motor state
% % all velocity, accel states are zero
% task_param.with_motor_dynamics = 1;
% task_param.multiple_inits = 0;
%----%

%task = OptTask(robot_model, 'fast_reach', 'net_mechpower', task_param);
%%%%

%%%% step 3: init OC handler

cost_param = [];
%cost_param.w0 = 1;

cost_param.w_e = 1e-4;
cost_param.w_t = 1e3;
cost_param.w_tf = cost_param.w_t*dt;
cost_param.w_r = 1e-5;
%cost_param.alpha = alpha;
cost_param.epsilon = 1e-6;
cost_param.T = T;
cost_param.dt = dt;
cost_param.target = target;
cost_param.fd = 1; % use finite difference or not
cost_param.x0 = x0;

% robot dynamics
f = @(x,u)robot_model.dynamics(x,u);
%disdyn = @(x,u)discrete_dynamics(f,x,u);
task1 = mccpvd1_reach(robot_model, cost_param);

% cost functions
%cost_param2=cost_param;
%cost_param2.w_e = cost_param.w_e*(1e-3);
%task2 = mccpvd1_reach(robot_model, cost_param2);
j1 = @(x,u,t)task1.j_effort(x,u,t);
j2 = @(x,u,t)task1.j_effort_rege(x,u,t);
%disj1 = @(x, u, t) discrete_cost(j1, x, u, t);
%disj2 = @(x, u, t) discrete_cost(j2, x, u, t);

dyncst1 =@(x, u, i) dyna_cost(f, j1, x, u, i);
dyncst2 = @(x, u, i)dyna_cost(f, j2, x, u, i);
% control limits
Op.lims = [robot_model.umin, robot_model.umax];
%Op.lambda = 0.01;
%Op.dlambda = 0.01;
%Op.lambdaMax = 1e20;
%Op.lambdaFactor = 1.2;
%full_DDP = false;
opt_param = [];
opt_param.umax = robot_model.umax; 
opt_param.umin = robot_model.umin;
opt_param.lambda_init = 0.01;
opt_param.lambda_max  = 1e20;
opt_param.iter_max = 300;
opt_param.online_plotting = 0;
opt_param.online_printing = 1;
opt_param.dcost_converge = 10^-9;
opt_param.solver = 'rk4';
opt_param.target = target;

opt_param.T = T;

%opt_param.umin(1) = target;
%opt_param.umax(1) = target;

%opt_param.umin(2) = x0(4);
%opt_param.umax(2) = x0(4);

% u0 can be full command sequence or just initial point
%u0 = result0.u;
u0 = [target; x0(4); 0];
%u0 = [0.5;0.1;0.2];
u0 = repmat(u0, 1, N-1);


%% critical damped
psim.dt = 0.02;
psim.solver = 'rk4';

result0.u = repmat([target; 0.1; 0],1,N-1);
result0.x = repmat(x0,1,N);
for i = 1:N-1
    result0.k(i) = robot_model.stiffness( result0.x(:,i) );
    result0.d(i) = min(robot_model.actuator.max_damping,max(0,2*sqrt(result0.k(i)*robot_model.inertia) - robot_model.Df));
    result0.u(3,i) = result0.d(i)/robot_model.actuator.max_damping;
    result0.x(:,i+1) = simulate_step(f,result0.x(:,i),result0.u(:,i),psim);
    
end
%% dynamic braking

%result1 = ILQRController.ilqr(f, j1, dt, N, x0, u0, opt_param);
result1 = iLQG_wrapper(dyncst1, x0, u0, Op);
for i = 1:N-1
    result1.k(i) = robot_model.stiffness( result1.x(:,i));
    result1.d(i) = robot_model.damping( result1.u(:,i));
    
end

%% pure regenerative braking
u3_maxrege = robot_model.actuator.u_max_regedamp;
opt_param2 = opt_param;
opt_param2.umax(3) = u3_maxrege;
Op.lims = [opt_param2.umin, opt_param2.umax];
%result2 = ILQRController.ilqr(f, j1, dt, N, x0, u0, opt_param2);
result2 = iLQG_wrapper(dyncst1, x0, u0, Op);
for i = 1:N-1
    result2.k(i) = robot_model.stiffness( result2.x(:,i));
    result2.d(i) = robot_model.damping( result2.u(:,i));
    
end
%% %% hybrid mode
opt_param3 = opt_param;
opt_param3.umax(3) = 0.75;
Op.lims = [opt_param3.umin, opt_param3.umax];
%result3 = ILQRController.ilqr(f, j2, dt, N, x0, u0, opt_param3);
result3 = iLQG_wrapper(dyncst2, x0, u0, Op);
for i = 1:N-1
    result3.k(i) = robot_model.stiffness( result3.x(:,i));
    result3.d(i) = robot_model.damping( result3.u(:,i));
    
end

%%
% result1 = ILQRController.ilqr(f, j1, dt, N, x0, result2.u, opt_param);
% 
% for i = 1:N-1
%     result1.k(i) = robot_model.stiffness( result1.x(:,i));
%     result1.d(i) = robot_model.damping( result1.u(:,i));
%     result1.Prege(i) = robot_model.power_rege( result1.x(:,i),result1.u(:,i));
% end

%% fixed max rege power damping
opt_param4 = opt_param;
opt_param4.umax(3) = u3_maxrege;
opt_param4.umin(3) = u3_maxrege;

Op.lims = [opt_param4.umin, opt_param4.umax];
%result4 = ILQRController.ilqr(f, j1, dt, N, x0, u0, opt_param4);
result4 = iLQG_wrapper(dyncst1, x0, u0, Op);
for i = 1:N-1
    result4.k(i) = robot_model.stiffness( result4.x(:,i));
    result4.d(i) = robot_model.damping( result4.u(:,i));
    
end

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

result4.usim = scale_controlSeq(result4.u,t(1:end-1),tsim(1:end-1));
result4.xsim = simulate_feedforward(x0,f,result4.usim,psim);

result0.usim = scale_controlSeq(result0.u,t(1:end-1),tsim(1:end-1));
result0.xsim = simulate_feedforward(x0,f,result0.usim,psim);

for i=1:length(tsim)-1
    result0.Prege(i) = robot_model.power_rege(result0.xsim(:,i),result0.usim(:,i));
    result1.Prege(i) = robot_model.power_rege( result1.xsim(:,i),result1.usim(:,i));
    result2.Prege(i) = robot_model.power_rege( result2.xsim(:,i),result2.usim(:,i));
    result3.Prege(i) = robot_model.power_rege( result3.xsim(:,i),result3.usim(:,i));
    result4.Prege(i) = robot_model.power_rege( result4.xsim(:,i),result4.usim(:,i));

end

%%
result0.Erege = sum(result0.Prege)*psim.dt;
result1.Erege = sum(result1.Prege)*psim.dt;
result2.Erege = sum(result2.Prege)*psim.dt;
result3.Erege = sum(result3.Prege)*psim.dt;
result4.Erege = sum(result4.Prege)*psim.dt;
for i = 1:length(tsim)-1
    result0.Plink(i) = robot_model.power_link(result0.xsim(:,i), result0.usim(:,i));
    result1.Plink(i) = robot_model.power_link(result1.xsim(:,i), result1.usim(:,i));
    result2.Plink(i) = robot_model.power_link(result2.xsim(:,i), result2.usim(:,i));
    result3.Plink(i) = robot_model.power_link(result3.xsim(:,i), result3.usim(:,i));
    result4.Plink(i) = robot_model.power_link(result4.xsim(:,i), result4.usim(:,i));
    
end
result0.E = sum(result0.Plink)*psim.dt;
result1.E = sum(result1.Plink)*psim.dt;
result2.E = sum(result2.Plink)*psim.dt;
result3.E = sum(result3.Plink)*psim.dt;
result4.E = sum(result4.Plink)*psim.dt;

%%
figure
subplot(2,2,1)

hold on

plot(t, result0.x(1,:),'--')
%plot(t, ones(1,N)*target)
plot(t, result1.x(1,:),'-','Color', [1 0.6 0.6], 'LineWidth', 3)
plot(t, result2.x(1,:),'b-.','LineWidth',1)
plot(t, result3.x(1,:),'k-','LineWidth',1)
plot(t, result4.x(1,:),'-','Color', [1 0.6 0.6], 'LineWidth', 1.5)
legend('C.D.','dynamic', 'regenerative','hybrid','fixed damping')
hold off

% equilibrium position
% subplot(2,3,2)
% hold on
% plot(t, result3.x(3,:),'-','Color', [1 0.6 0.6], 'LineWidth', 1.5)
% 
% plot(t, result0.x(3,:),'--')
% plot(t, result1.x(3,:),'r-','LineWidth',1)
% plot(t, result2.x(3,:),'b-.','LineWidth',1)
% hold off
% stiffness
subplot(2,2,2)
hold on

plot(t(1:end-1), result0.k,'--')
plot(t(1:end-1), result1.k,'-','Color', [1 0.6 0.6], 'LineWidth', 3)
plot(t(1:end-1), result2.k,'b-.','LineWidth',1)
plot(t(1:end-1), result3.k,'k-','LineWidth',1)
plot(t(1:end-1), result4.k,'-','Color', [1 0.6 0.6], 'LineWidth', 1.5)

hold off
% damping
subplot(2,2,3)
hold on

plot(t(1:end-1), result0.d,'--')
plot(t(1:end-1), result1.d,'-','Color', [1 0.6 0.6], 'LineWidth', 3)
plot(t(1:end-1), result2.d,'b-.','LineWidth',1)
plot(t(1:end-1), result3.d,'k-','LineWidth',1)
plot(t(1:end-1), result4.d,'-','Color', [1 0.6 0.6], 'LineWidth', 1.5)
hold off

subplot(2,2,4)
hold on
c = categorical({'C.D.','dynamic','regenerative','hybrid','fixed damp'});
E = [result0.E, result1.E, result2.E, result3.E, result4.E];
Erege = [result0.Erege, 0, result2.Erege, result3.Erege, result4.Erege];
Enet = E - Erege;
zeta = Erege./E;
bar(c, E, 'FaceColor',[0.5 0 .5],'EdgeColor',[0.9 0 .9],'LineWidth',1.0)
bar(c, Enet, 'FaceColor',[0 .5 .5],'EdgeColor',[0 .9 .9],'LineWidth',1.0)
hold off
%%
% uu3 = 0:0.01:1; 
% for i=1:length(uu3)
%     dd(i) = robot_model.actuator.damping(uu3(i)); 
%     pp(i) = robot_model.actuator.power_rege(1,uu3(i));
% end
% figure 
% plot(dd, pp)
figure
subplot(3,1,1)
plot(t(1:end-1), result0.u(1,:),'--')
plot(t(1:end-1), result1.u(1,:),'-','Color', [1 0.6 0.6], 'LineWidth', 3)
plot(t(1:end-1), result2.u(1,:),'b-.','LineWidth',1)
plot(t(1:end-1), result3.u(1,:),'k-','LineWidth',1)
plot(t(1:end-1), result4.u(1,:),'-','Color', [1 0.6 0.6], 'LineWidth', 1.5)

subplot(3,1,2)
plot(t(1:end-1), result0.u(2,:),'--')
plot(t(1:end-1), result1.u(2,:),'-','Color', [1 0.6 0.6], 'LineWidth', 3)
plot(t(1:end-1), result2.u(2,:),'b-.','LineWidth',1)
plot(t(1:end-1), result3.u(2,:),'k-','LineWidth',1)
plot(t(1:end-1), result4.u(2,:),'-','Color', [1 0.6 0.6], 'LineWidth', 1.5)

subplot(3,1,3)
plot(t(1:end-1), result0.u(3,:),'--')
plot(t(1:end-1), result1.u(3,:),'-','Color', [1 0.6 0.6], 'LineWidth', 3)
plot(t(1:end-1), result2.u(3,:),'b-.','LineWidth',1)
plot(t(1:end-1), result3.u(3,:),'k-','LineWidth',1)
plot(t(1:end-1), result4.u(3,:),'-','Color', [1 0.6 0.6], 'LineWidth', 1.5)


function [result] = iLQG_wrapper(DYNCST, x0, u0, Op)
[x, u, L, Vx, Vxx, cost, trace, stop] = iLQG(DYNCST, x0, u0, Op);
result.x = x;
result.u = u;
result.L = L;
result.Vx = Vx;
result.Vxx = Vxx;
result.cost = cost;
result.trace = trace;
result.stop = stop;
end
function J = finite_difference(fun, x, h)
% simple finite-difference derivatives
% assumes the function fun() is vectorized

if nargin < 3
    h = 2^-17;
end

[n, K]  = size(x);
H       = [zeros(n,1) h*eye(n)];
H       = permute(H, [1 3 2]);
X       = pp(x, H);
X       = reshape(X, n, K*(n+1));
Y       = fun(X);
m       = numel(Y)/(K*(n+1));
Y       = reshape(Y, m, K, n+1);
J       = pp(Y(:,:,2:end), -Y(:,:,1)) / h;
J       = permute(J, [1 3 2]);
end
function c = pp(a,b)
c = bsxfun(@plus,a,b);
end

function c = tt(a,b)
c = bsxfun(@times,a,b);
end



function c = discrete_cost(l, x, u, i)

cc = l(x, u, i*0.02);
if isnan(u)
    c = cc;
else
    c=cc*0.02;
end

end

function c = reaching_cost(x, u)
final = isnan(u(1,:));
u(:,final)  = 0;
c = (pi/4 - x).^2 + u.^2;
end