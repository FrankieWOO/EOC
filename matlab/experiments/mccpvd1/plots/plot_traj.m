
function []=plot_traj(x,u,p)
% plot single or a series of trajectories. Multiple trajs are stored in
% cell array. Input x and u can be a numeric vector or column cell array.


if ~iscell(x)
    tempx = cell(1,1);
    tempu = cell(1,1);
    tempx{1} = x;
    tempu{1} = u;
    x = tempx;
    u = tempu;
    Nx =1;
else
    Nx = size(x,1);
end

%     if isfield(p, 'power_model')
%         power_model = p.power_model;
%     else
%         power_model = 8;
%     end
% The number of time steps
Nt = size(x{1},2); dt=p.T/Nt; t=(0:Nt-1)*dt;
% The number of x

    model = p.model;
    if isfield(p, 'recovery_ratio')
        ratio = 1 - p.recovery_ratio;
        recovery_ratio = p.recovery_ratio;
    else
        ratio = 1;
        recovery_ratio = 0;
    end
    kappa = model.spring_constant; 
    r = model.drum_radius;
    C = model.pin_displacement; B = model.lever_length; L0 = abs(C-B);
    power_motor1 = cell(Nx,1);
    power_motor2 = cell(Nx,1);
    power = cell(Nx,1);
    for i=1:Nx
    A = sqrt(B^2+C^2 - 2*B*C*cos(x{i}(3,1:end-1)-x{i}(1,1:end-1)));
    %L: spring length
    L = r*x{i}(6,1:end-1) + A - L0;
    tau_motor1 = kappa*B*C*sin(x{i}(3,1:end-1)-x{i}(1,1:end-1))*(1+ (r*x{i}(6,1:end-1)-L0)/A );
            tau_motor2 = kappa*L*r;
            power_motor1{i} = tau_motor1.*x{i}(4,1:end-1);
            power_motor2{i} = tau_motor2.*x{i}(7,1:end-1);
            power_motor1{i}(power_motor1{i}<0) = 0;
            power_motor2{i}(power_motor2{i}<0) = 0;
            damp = zeros(1,Nt-1);
            for j=1:Nt-1
                damp(j) = model.getDamp(x{i}(:,j),u{i}(:,j));
            end
            power{i} = power_motor1{i} + power_motor2{i} - damp.*x{i}(2,1:end-1).^2*(1-ratio);
    end


%settling time
st = zeros(Nx,1);

figure
%set(groot, 'defaultAxesTickLabelInterpreter','latex'); 
%set(groot, 'defaultLegendInterpreter','latex');
subplot(5,2,1)
% position
hold on
for k=1:Nx
    plot(t,x{k}(1,:))
    st(k) = settling_time(p.x_target,x{k}(1,:),x{k}(2,:),t);
    if ~isnan(st(k))
        plot(st(k),x{k}(1,round(st(k)/dt+1)),'-x')
    end
end
plot(t,ones(1,Nt)*p.x_target,'k--')
xlabel('t (s)','interpreter','latex')
ylabel('$q$ (rad)','interpreter','latex')
%legend('0 energy weight','0.05 energy weight','50% energy recovery')
hold off

subplot(5,2,2)
% velocity
hold on
for k=1:Nx
    plot(t,x{k}(2,:))
end
xlabel('t (s)','interpreter','latex')
ylabel('$\dot{q}$ (rad/s)','interpreter','latex')
%legend('0 energy weight','0.05 energy weight','50% energy recovery')
hold off

subplot(5,2,3)
hold on
for k=1:Nx
    plot(t,x{k}(3,:))
end
xlabel('t (s)','interpreter','latex')
ylabel('$\theta_1$ (rad)','interpreter','latex')
%legend('0 energy weight','0.05 energy weight','50% energy recovery')
plot(t,ones(1,Nt)*p.x_target,'k--')
hold off

subplot(5,2,5)
hold on
xlabel('t (s)','interpreter','latex')
ylabel('$u_1$','interpreter','latex')
for k=1:Nx
    plot(t(1:end-1),u{k}(1,:))
end
%legend('0 energy weight','0.05 energy weight','50% energy recovery')
plot(t(1:end-1),ones(1,Nt-1)*p.x_target,'k--')
hold off

subplot(5,2,4)
hold on
for k=1:Nx
    plot(t,x{k}(6,:))
end
xlabel('t (s)','interpreter','latex')
ylabel('$\theta_2$ (rad)','interpreter','latex')
%legend('0 energy weight','0.05 energy weight','50% energy recovery')
hold off

subplot(5,2,6)
hold on
for k=1:Nx
    plot(t(1:end-1),u{k}(2,:))
end
xlabel('t (s)')
ylabel('$u_2$','interpreter','latex')
%legend('0 energy weight','0.05 energy weight','50% energy recovery')
hold off

subplot(5,2,10)
hold on
for k=1:Nx
    plot(t(1:end-1),u{k}(3,:))
end
xlabel('t (s)','interpreter','latex')
ylabel('$u_3$','interpreter','latex')
hold off
%legend('0 energy weight','0.05 energy weight','50% energy recovery')

subplot(5,2,9)
hold on
for k=1:Nx
powers = zeros(1,Nt-1);
for j=1:Nt-1
    powers(j) = mw_power_mcc(x{k}(:,j),u{k}(:,j),p);
end
plot(t(1:end-1),powers)
end
xlabel('t (s)','interpreter','latex')
ylabel('$p_m$ (W)','interpreter','latex')
hold off

subplot(5,2,7)
hold on
for k=1:Nx
    plot(t(1:end-1),power_motor1{k})
end
xlabel('t (s)','interpreter','latex')
ylabel('$p_{m_1}$ (W)','interpreter','latex')
hold off

subplot(5,2,8)
hold on
for k=1:Nx
    plot(t(1:end-1),power_motor2{k})
end
xlabel('t (s)','interpreter','latex')
ylabel('$p_{m_2}$ (W)','interpreter','latex')
hold off

end
