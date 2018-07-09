function [ h ] = plot_traj_mccpvd1( traj )
%   PLOT_TRAJ Summary of this function goes here
%   Detailed explanation goes here

    %h=[];
    x = traj.x;
    u = traj.u;
    t = traj.t;
    
    fig1 = figure();
    title('trajectory')
    subplot(2,1,1)
    hold on
    plot(t, x(1,:))
    
    if isfield(traj, 'target')
        plot(t, ones( size( x(1,:)))*traj.target, '--');
    end
    if isfield(traj, 'xsim')
        plot(traj.tsim, traj.xsim(1,:));
    end
    xlabel('time')
    ylabel('position')
    hold off
    
    subplot(2,1,2)
    hold on
    plot(t,x(2,:))
    xlabel('time')
    ylabel('velocity')
    hold off
    h = fig1;
    
    % control commands for motors
    fig2 = figure();
    subplot(3,1,1)
    hold on
    plot(t(1:end-1), u(1,:))
    plot(t(1:end-1), x(3,1:end-1))
    ylabel('motor 1')
    legend('u1','\theta_1')
    hold off
    subplot(3,1,2)
    hold on
    plot(t(1:end-1), u(2,:))
    plot(t(1:end-1), x(4,1:end-1))
    hold off
    ylabel('motor 2')
    legend('u2','\theta_2')
    subplot(3,1,3)
    plot(t(1:end-1), u(3,:))
    ylabel('u3')
    xlabel('time')
    h = [h;fig2];
    
    % power and energy
    if isfield(traj, 'power_in')
    fig3 = figure();
    subplot(2,1,1)
    Ntsim = length(traj.tsim);
    hold on
    plot(traj.tsim(1:end-1),traj.power_in)
    plot(traj.tsim(1:end-1),traj.power_in1)
    plot(traj.tsim(1:end-1),traj.power_in2)
    plot(traj.tsim(1:end-1),traj.power_out)
    plot(traj.tsim(1:end-1),traj.power_damp)
    plotyy(traj.tsim(1:end-1),traj.power_rege,traj.tsim(1:Ntsim-1),traj.Es(1:Ntsim-1))
    
    hold off
    legend('p_{\mathrm{in}}','p_{\mathrm{in1}}','p_{\mathrm{in2}}',...
        'p_{\mathrm{out}}','p_{\mathrm{damp}}','p_{\mathrm{rege}}','E_s')
    subplot(2,1,2)
    c2plt = categorical({'E_{\mathrm{in1}}','E_{\mathrm{in2}}',...
        'E_{\mathrm{in}}','E_{\mathrm{out}}','E_{\mathrm{damp}}','E_{\mathrm{rege}}'});
    E2plt = [ traj.Ein1; traj.Ein2; traj.Ein; traj.Eout; traj.Edamp; traj.Erege ];
    bar(c2plt,E2plt)
    
    h = [h;fig3];
    end
    
    
end

