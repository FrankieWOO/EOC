function [ y ] = exetraj_plot( u )
%EXETRAJ_PLOT Summary of this function goes here
%   Detailed explanation goes here
    y = exetraj(u);
    
    dt = 0.02;
    Nt = size(y,2);
    T = (Nt-1)*dt;
    t = 0:dt:T;
    
    fig = figure();
    title('recorded')
    subplot(2,1,1)
    plot(t, y(1,:) )
    title('position')
    
    subplot(2,1,2)
    plot(t, y(2,:) )
    title('rege power')
    
end

