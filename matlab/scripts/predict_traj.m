function [ y ] = predict_traj( model, x0, u, t )
%PREDICT_TRAJ Summary of this function goes here
%   Detailed explanation goes here
    %tsim = 0:0.001:t(end);
    %usim = scale_controlSeq(u,t(1:end-1),tsim(1:end-1));
    psim.solver = 'ode45';
    %psim.dt = 0.001;
    psim.dt = 0.02;
    %xsim = simulate_feedforward(x0,model,usim,psim);
    f = @(x,u)model.dynamics(x,u);
    xsim = simulate_feedforward(x0, f, u, psim);
    Prege = zeros(1,length(t));
    for i=1:length(t)-1
        Prege(i) = model.power_rege(xsim(:,i),u(:,i));
    end
    Prege(end) = Prege(end-1);
    y = [xsim; Prege];
end

