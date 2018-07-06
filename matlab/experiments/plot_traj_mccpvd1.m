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
    xlabel('time')
    ylabel('position')
    hold off
    
    subplot(2,1,2)
    hold on
    plot(t,x(2,:))
    xlabel('time')
    ylabel('velocity')
    hold off
    
    fig2 = figure();
    subplot(3,1,1)
    plot(t(1:end-1), u(1,:))
    ylabel('u1')
    subplot(3,1,2)
    plot(t(1:end-1), u(2,:))
    ylabel('u2')
    subplot(3,1,3)
    plot(t(1:end-1), u(3,:))
    ylabel('u3')
    xlabel('time')
    
    h = [fig1;fig2];
end

