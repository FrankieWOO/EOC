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
            data.power_rege = DataProcess.compute_power_rege(data.rege_current);
            data.Erege = DataProcess.compute_Erege(data.rege_current, data.header);
            data.settle_time = DataProcess.settle_time(data.v, data.header);
            
        end
        
        function [data] = preprocess_single_traj2(data)
            data = DataProcess.trim_head(data);
            data.p = DataProcess.central_difference_smooth(data.joint_position,5);
            data.v = DataProcess.compute_velocity_centraldiff(data.p, data.header);
            data.acc = DataProcess.compute_accel_centraldiff(data.v, data.header);
            data.power_rege = data.rege_current/1000;
            data.Erege = DataProcess.integrate(data.power_rege, data.header);
            data.settle_time = DataProcess.settle_time(data.v, data.header);
            
        end
        
        function [] = plot_single_traj(data)
            
            plot(data.header, data.joint_position)
        end
        
        
        function stats = compute_trajs_stats(data)
            stats.accuracy_score = 0;
            stats.accuracy_halt = 0;
            stats.avg_settle_time = 0;
            stats.avg_overshot = 0;
            stats.total_Erege = 0;
            stats.total_Ein = 0;
            stats.pcnt_Erege = 0;
            stats.total_kinetic = 0;
            stats.Ein = 0;
            stats.predict_Erege=0;
            for i=1:size(data,1)
                for j=1:size(data,2)
                stats.accuracy_score = stats.accuracy_score + data{i,j}.accuracy_score;
                stats.accuracy_halt = stats.accuracy_halt + data{i,j}.accuracy_halt;
                stats.avg_settle_time = stats.avg_settle_time + data{i,j}.settle_time;
                stats.total_Erege = stats.total_Erege + data{i,j}.Erege;
                stats.total_kinetic = stats.total_kinetic + data{i,j}.max_kinetic;
                stats.avg_overshot = stats.avg_overshot + data{i,j}.overshot;
                stats.Ein = stats.Ein + data{i,j}.Ein;
                stats.predict_Erege = stats.predict_Erege + data{i,j}.predict_Erege;
                end
            end
            stats.accuracy_score = stats.accuracy_score/(i*j);
            stats.accuracy_halt = stats.accuracy_halt/(i*j);
            stats.avg_settle_time = stats.avg_settle_time/(i*j);
            stats.avg_overshot = stats.avg_overshot/(i*j);
            stats.avg_Erege = stats.total_Erege/(i*j);
            stats.avg_kinetic = stats.total_kinetic/(i*j);
            stats.pcnt_Erege = stats.avg_Erege/stats.avg_kinetic;
            stats.avg_Ein = stats.Ein/(i*j);
            stats.avg_predict_Erege = stats.predict_Erege/(i*j);
        end
        
        % plot a list of trajs
        % input: data - cell array of trajs
        function [] = plot_trajs(data, target_list)
            p = [];
            v = [];
            acc = [];
            t = [];
            power_rege=[];
            % merge data
            segments = zeros(length(data),1);
            time_end_ind = 0;
            last_time_end = 0;
            target_lines = [];
            for i = 1:length(data)
                p = [p;data{i}.p];
                v = [v;data{i}.v];
                acc = [acc;data{i}.acc];
                t = [t;data{i}.header+last_time_end];
                last_time_end = last_time_end + data{i}.header(end);
                power_rege = [power_rege;data{i}.power_rege];
                time_end_ind = time_end_ind + length(data{i}.header);
                segments(i) = time_end_ind;
                
                
                target_lines = [target_lines; ones(size(data{i}.p))*target_list(i)];
                
            end
            
            
            
            
            figure
            subplot(3,1,1)
            hold on
            plot(t, p, 'LineWidth', 1)
            plot(t, target_lines, '--')
            yL = get(gca, 'YLim');
            for k=1:length(data)
                line([t(segments(k)) t(segments(k))], yL, 'LineStyle','--','Color',[0.5 0.5 0.5])
            end
            xlabel('time (s)')
            ylabel('position (rad)')
            hold off
            
            subplot(3,1,2)
            hold on
            plot(t, v,'LineWidth', 1)
            yL = get(gca, 'YLim');
            for k=1:length(data)
                line([t(segments(k)) t(segments(k))], yL, 'LineStyle','--','Color',[0.5 0.5 0.5])
            end
            xlabel('time (s)')
            ylabel('velocity (rad/s)')
            hold off
            
            subplot(3,1,3)
            hold on
            plot(t, power_rege,'LineWidth', 1)
            yL = get(gca, 'YLim');
            for k=1:length(data)
                line([t(segments(k)) t(segments(k))], yL, 'LineStyle','--','Color',[0.5 0.5 0.5])
            end
            xlabel('time (s)')
            ylabel('power (W)')
            hold off
            
        end
        
        function [p,v,acc,t,power_rege,segments,target_lines]=merge_traj_list(data, target_list, duration)
            
            p = [];
            v = [];
            acc = [];
            t = [];
            power_rege=[];
            % merge data
            segments = zeros(length(data),1);
            time_end_ind = 0;
            last_time_end = 0;
            target_lines = [];
            for i = 1:length(data)
                try
                    ind_end = findFirst(data{i}.header,duration)-1;
                catch
                    ind_end = length(data{i}.header);
                end
                if isnan(ind_end), ind_end = length(data{i}.header); end
                p = [p;data{i}.p(1:ind_end)];
                v = [v;data{i}.v(1:ind_end)];
                acc = [acc;data{i}.acc(1:ind_end)];
                t = [t;data{i}.header(1:ind_end)+last_time_end];
                %last_time_end = last_time_end + data{i}.header(end);
                last_time_end = last_time_end + duration;
                
                power_rege = [power_rege;data{i}.power_rege(1:ind_end)];
                time_end_ind = time_end_ind + ind_end;
                segments(i) = time_end_ind;
                
                
                target_lines = [target_lines; ones(size(data{i}.p(1:ind_end)))*target_list(i)];
                
            end
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
            
            v(end-9:end) = DataProcess.mavg_filter(v(end-9:end), 3);
        end
        
        function [v] = compute_velocity_forwarddiff( p, t )
            % central difference and central window smoothing;
            v = zeros(size(p));
            %a = 0.8;
            v(1) = 0 ;
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
            
%             window = 7;
%             for j = 1:window
%                 acc(j) = mean(acc(1:2*j-1));
%             end
%             
%             for k = window+1:length(acc)-window
%                 acc(k) = mean(acc(k-window:k+window));
%             end
%             
%             for k = length(acc)-window+1:length(acc)
%                 w = length(acc)-k;
%                 acc(k) = mean( acc( k-w:k+w ) );
%             end
              
            acc = DataProcess.central_difference_smooth(acc, 7);
            acc(end-20:end) = DataProcess.mavg_filter(acc(end-20:end), 5);
            
        end
        
        function [st] = settle_time(v, t)
            % 
            st = t(end);
            halt = false;
            acc = DataProcess.compute_accel_centraldiff(v, t);
            
            [~, ind] = max(v);
            
            for k = ind:length(t)
                if (abs(v(k)) < 0.1) && (abs(acc(k)) < 3)
                    st = t(k);
                    for j=k+1:min(k+25,length(t))
                        if (abs(v(k)) < 0.2) 
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
        
        function E = compute_Erege(x, t)
            p = DataProcess.compute_power_rege(x);
            E = DataProcess.integrate(p, t);
        end
        
        function Prege = compute_power_rege(x)
            x = x/1000;
            Prege = 25.3*x.^2;
        end
        
        function Y = integrate(x, t)
            dt = diff(t);
            y = (x(1:end-1) + x(2:end))/2;
            Y = sum(y.*dt);
        end
        
        function score = compute_accuracy_score(x, t, target)
            dt = mean(diff(t));
            y = (x - target).^2;
            score = sum(dt*y);
        end
        
        function score = compute_accuracy_after_halt(traj, target)
            p = traj.p;
            t = traj.header;
            st = traj.settle_time;
            st_ind = findFirst(t, st)-1;
            if isnan(st_ind)
                st_ind = length(t);
            end
            dt = mean(diff(t));
            if length(t)-st_ind < 24
                x = p(end-24:end);
            else
                x = p(st_ind: st_ind+24);
            
            end
            y = (x-target).^2;
            score = sum(dt*y);
        end
        
        function out = compute_overshot(traj, target)
            t = traj.header;
            settle_time = traj.settle_time;
            settle_time_ind = findFirst(t, settle_time)-1;
            if isnan(settle_time_ind), settle_time_ind = length(t);end
            
            
            shot_time_ind = 0;
            offset = traj.p - target;
            

            
            for i=2:settle_time_ind
                if offset(i)*offset(i-1) < 0
                    shot_time_ind = i;
                    break;
                end
            end
            
            if shot_time_ind == 0
                out = 0;
            else
                dt = mean(diff(t(shot_time_ind:settle_time_ind))) ;
                out = sum(offset(shot_time_ind:settle_time_ind).^2*dt);
            end
        end
    end
    
end

