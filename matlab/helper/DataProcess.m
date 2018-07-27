classdef DataProcess
    %DATAPROCESS Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
    end
    
    methods
        function obj = DataProcess(varargin)
            
        end
    end
    
    methods (Static)
        function [y] = exp_filter(x, a)
            if nargin == 1
                a = 0.8;
            end
            y = zeros(size(x));
            y(1) = x(1);
            
            for i = 2:length(x)
                y(i) = a*y(i-1) + (1-a)*x(i);
            end
        end
        
        
        function [y] = mavg_filter(x, window_size)
            if nargin == 1
               window_size = 3; 
            end
            
            y = filter( (1/window_size)*ones(1,window_size), 1, x);
            y(1:window_size-1) = x(1:window_size-1);
        end
        
        function [y] = central_difference_smooth(x, window)
            y = x;
            if nargin == 1
                window = 1;
            end
            for j = 1:window
                y(j) = mean(x(1:2*j-1));
            end
            
            for k = window+1:length(x)-window
                y(k) = mean(x(k-window:k+window));
            end
            
            for k = length(x)-window+1:length(x)
                w = length(x)-k;
                y(k) = mean( x( k-w:k+w ) );
            end
        end
        
        
        % trim head: trim extra data points before t=0, put the last
        % negative timestamp to 0
        function [data] = trim_head(data)
            ind = findFirst(data.header, 0);
            if ind == 2
                data.header(1) = 0;
            end
            
            if ind > 2
                data.header(ind-1) = 0;
                
                fields = fieldnames(data);
                for i = 1:length(fields)
                    data.(fields{i})(1:ind-2) = [];
                end
                
            end
            
        end
        
        % preprocess single traj recording
        function [data] = preprocess_single_traj(data)
            data = DataProcess.trim_head(data);
            data.p = DataProcess.central_difference_smooth(data.joint_position,5);
            data.v = DataProcess.compute_velocity_centraldiff(data.p, data.header);
            data.acc = DataProcess.compute_accel_centraldiff(data.v, data.header);
            data.Erege = DataProcess.compute_Erege(data.rege_current, data.header);
            data.power_rege = DataProcess.compute_power_rege(data.rege_current);
        end
        
        function [] = plot_single_traj(data)
            
            plot(data.header, data.joint_position)
        end
        
        % input: sequence of position
        % output: sequence of velocity
        % by default, the first entry of velocity is set to 0
        function [v] = compute_velocity_backdiff( p, t )
            v = zeros(size(p));
            a = 0.8;
            for i = 2:length(p)
                v(i) = a*v(i-1) + (1-a)*(p(i) - p(i-1))/(t(i)- t(i-1));
            end
            
            v = DataProcess.mavg_filter(v,3);
        end
        
        function [v] = compute_velocity_centraldiff( p, t )
            % central difference and central window smoothing;
            v = zeros(size(p));
            
            for i = 2:length(p)-1
                v(i) = (p(i+1) - p(i-1))/(t(i+1)- t(i-1));
            end
            v(end) = (p(end) - p(end-1))/(t(end)- t(end-1));
            
            window = 5;
            for j = 1:window
                v(j) = mean(v(1:2*j-1));
            end
            
            for k = window+1:length(v)-window
                v(k) = mean(v(k-window:k+window));
            end
            
            for k = length(v)-window+1:length(v)
                w = length(v)-k;
                v(k) = mean( v( k-w:k+w ) );
            end
            
        end
        
        function [v] = compute_velocity_forwarddiff( p, t )
            % central difference and central window smoothing;
            v = zeros(size(p));
            %a = 0.8;
            v(1) = 0;
            for i = 2:length(p)-1
                v(i) = (p(i+1) - p(i))/(t(i+1)- t(i));
            end
            v(end) = (p(end) - p(end-1))/(t(end)- t(end-1));
            
            window = 3;
            for j = 1:window
                v(j) = mean(v(1:2*j-1));
            end
            
            for k = window+1:length(v)-window
                v(k) = mean(v(k-window:k+window));
            end
            
            for k = length(v)-window+1:length(v)
                w = length(v)-k;
                v(k) = mean( v( k-w:k+w ) );
            end
            
        end
        
        function acc = compute_accel_centraldiff(v,t)
            acc = zeros(size(v));
            
            for i = 2:length(t)-1
                acc(i) = (v(i+1) - v(i-1))/(t(i+1)- t(i-1));
            end
            acc(end) = (v(end) - v(end-1))/(t(end)- t(end-1));
            
            window = 7;
            for j = 1:window
                acc(j) = mean(acc(1:2*j-1));
            end
            
            for k = window+1:length(acc)-window
                acc(k) = mean(acc(k-window:k+window));
            end
            
            for k = length(acc)-window+1:length(acc)
                w = length(acc)-k;
                acc(k) = mean( acc( k-w:k+w ) );
            end
        end
        
        function [st] = settle_time(v, t)
            % 
            st = t(end);
            halt = false;
            acc = DataProcess.compute_accel_centraldiff(v, t);
            for k = 1:length(t)
                if (abs(v(k)) < 0.1) && (abs(acc(k)) < 1)
                    st = t(k);
                    for j=k+1:min(k+30,length(t))
                        if (abs(v(k)) < 0.1) && (abs(acc(k)) < 1)
                            halt = true;
                        else
                            halt = false;
                        end
                    end
                    if halt == true
                        break;
                    else
                        st = t(end);
                        
                    end
                end
            end
            
        end
        
        function [p] = filter_rege_current(p)
            for k = 1:length(p)
                if p(k) < 1.5
                    p(k) = 0;
                else
                    p(k) = p(k) - 1;
                end
            end
        end
        
        function E = compute_Erege(p, t)
            p = p/1000;
            E = DataProcess.integrate(25*p.^2, t);
        end
        
        function Prege = compute_power_rege(x)
            x=x/1000;
            Prege = 25*x.^2;
        end
        
        function Y = integrate(x, t)
            dt = diff(t);
            y = (x(1:end-1) + x(2:end))/2;
            Y = sum(y.*dt);
        end
        
    end
    
end

