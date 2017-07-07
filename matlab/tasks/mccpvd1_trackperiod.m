classdef mccpvd1_trackperiod
    %MCCPVD1_TRACKPERIOD tracking periodic movements
    
    properties
        T
        freq
        dt
        x_ref
        Nu
        w_t
        w_e
        w_r
        robot_model
    end
    
    methods
        function obj = mccpvd1_trackperiod(robot_model, p)
            obj.robot_model = robot_model;
            if isfield(p, 'T'), obj.T = p.T; end
            if isfield(p, 'dt'), obj.dt = p.dt; end
            if isfield(p, 'freq'), obj.freq = p.freq;end
            if isfield(p, 'x_ref'), obj.x_ref = p.x_ref;end
            if isfield(p, 'w_t'), obj.w_t = p.w_t; end
            if isfield(p, 'w_e'), obj.w_e = p.w_e; end
            if isfield(p, 'w_r'), obj.w_r = p.w_r; end
            if isfield(p, 'Nu'), obj.Nu = p.Nu; end
            
        end
        function [l, l_x, l_xx, l_u, l_uu, l_ux] = j_elec(obj, x, u, t)
            if (isnan(u))
                % final cost
                fl = @(x)obj.l_f(x);
                l = fl(x);
                if nargout>1
                    flJ = @(x) get_jacobian_fd(fl, x);
                    l_x = flJ(x);
                    l_xx = get_hessian_fd(flJ,x);
                end
            else
                fl = @(x,u,t) obj.l_elec(x,u,t);
                l = fl(x,u,t);
                
                
                if nargout>1
                    
                    
                    % finite difference
                    flJ=@(x,u,t)J_cost_fd ( fl, x, u, t );
                    [l_x ,l_u      ] = flJ ( x, u, t );
                    flH =@(x,u,t)H_cost_fd  ( flJ, x, u, t );
                    [l_xx,l_uu,l_ux] = flH  ( x, u, t );
                end
                
            end
            
        end
        function [l, l_x, l_xx, l_u, l_uu, l_ux] = j_elec_rege(obj, x, u, t)
            if (isnan(u))
                % final cost
                fl = @(x) obj.l_f(x);
                l = fl(x);
                if nargout>1
                    flJ = @(x) get_jacobian_fd(fl, x);
                    l_x = flJ(x);
                    l_xx = get_hessian_fd(flJ,x);
                end
            else
                fl = @(x,u,t) obj.l_elec_rege(x,u,t);
                l = fl(x,u,t);
                
                
                if nargout>1
                    
                    
                    % finite difference
                    flJ=@(x,u,t)J_cost_fd ( fl, x, u, t );
                    [l_x ,l_u      ] = flJ ( x, u, t );
                    flH =@(x,u,t)H_cost_fd  ( flJ, x, u, t );
                    [l_xx,l_uu,l_ux] = flH  ( x, u, t );
                end
                
            end
            
        end
        
        function [l, l_x, l_xx, l_u, l_uu, l_ux] = j_elec_posi(obj, x, u, t)
            if (isnan(u))
                % final cost
                fl = @(x)obj.l_f(x);
                l = fl(x);
                if nargout>1
                    flJ = @(x) get_jacobian_fd(fl, x);
                    l_x = flJ(x);
                    l_xx = get_hessian_fd(flJ,x);
                end
            else
                fl = @(x,u,t) obj.l_elec_posi(x,u,t);
                l = fl(x,u,t);
                
                
                if nargout>1
                    
                    
                    % finite difference
                    flJ=@(x,u,t)J_cost_fd ( fl, x, u, t );
                    [l_x ,l_u      ] = flJ ( x, u, t );
                    flH =@(x,u,t)H_cost_fd  ( flJ, x, u, t );
                    [l_xx,l_uu,l_ux] = flH  ( x, u, t );
                end
                
            end
            
        end
        function [l, l_x, l_xx, l_u, l_uu, l_ux] = j_elec_posi_rege(obj, x, u, t)
            if (isnan(u))
                % final cost
                fl = @(x) obj.l_f(x);
                l = fl(x);
                if nargout>1
                    flJ = @(x) get_jacobian_fd(fl, x);
                    l_x = flJ(x);
                    l_xx = get_hessian_fd(flJ,x);
                end
            else
                fl = @(x,u,t) obj.l_elec_posi_rege(x,u,t);
                l = fl(x,u,t);
                
                
                if nargout>1
                    
                    
                    % finite difference
                    flJ=@(x,u,t)J_cost_fd ( fl, x, u, t );
                    [l_x ,l_u      ] = flJ ( x, u, t );
                    flH =@(x,u,t)H_cost_fd  ( flJ, x, u, t );
                    [l_xx,l_uu,l_ux] = flH  ( x, u, t );
                end
                
            end
            
        end
        
        function [l, l_x, l_xx, l_u, l_uu, l_ux] = j_effort(obj, x, u, t)
            if (isnan(u))
                % final cost
                fl = @(x) obj.l_f(x);
                l = fl(x);
                if nargout>1
                    flJ = @(x) get_jacobian_fd(fl, x);
                    l_x = flJ(x);
                    l_xx = get_hessian_fd(flJ,x);
                end
            else
                fl = @(x,u,t) obj.l_effort(x,u,t);
                l = fl(x,u,t);
                
                
                if nargout>1
                    
                    
                    % finite difference
                    flJ=@(x,u,t)J_cost_fd ( fl, x, u, t );
                    [l_x ,l_u      ] = flJ ( x, u, t );
                    flH =@(x,u,t)H_cost_fd  ( flJ, x, u, t );
                    [l_xx,l_uu,l_ux] = flH  ( x, u, t );
                end
                
            end
            
        end
        function [l, l_x, l_xx, l_u, l_uu, l_ux] = j_effort_rege(obj, x, u, t)
            if (isnan(u))
                % final cost
                fl = @(x) obj.l_f(x);
                l = fl(x);
                if nargout>1
                    flJ = @(x) get_jacobian_fd(fl, x);
                    l_x = flJ(x);
                    l_xx = get_hessian_fd(flJ,x);
                end
            else
                fl = @(x,u,t) obj.l_effort_rege(x,u,t);
                l = fl(x,u,t);
                
                
                if nargout>1
                    
                    
                    % finite difference
                    flJ=@(x,u,t)J_cost_fd ( fl, x, u, t );
                    [l_x ,l_u      ] = flJ ( x, u, t );
                    flH =@(x,u,t)H_cost_fd  ( flJ, x, u, t );
                    [l_xx,l_uu,l_ux] = flH  ( x, u, t );
                end
                
            end
            
        end
        function cost = l_effort(obj, x, u, t)
            n = round(t/obj.dt) + 1;
            track_error = obj.L2_error(obj.x_ref(:,n), x(1:2));
            
            energy_cost = norm(u(1:2) - x(3:4),2) ;
            %energy_cost = (u(1) - self.target)^2 + u(2)^2;
            cost = track_error*obj.w_t + energy_cost*obj.w_e;
        end
        function cost = l_effort_rege(obj, x, u, t)
            n = round(t/obj.dt) + 1;
            track_error = obj.L2_error(obj.x_ref(:,n), x(1:2));
            
            energy_cost = norm(u(1:2) - x(3:4),2) ;
            p_rege = obj.robot_model.power_rege(x,u);
            cost = track_error*obj.w_t + energy_cost*obj.w_e - p_rege*obj.w_r;
        end
        function cost = l_elec(obj, x, u, t)
            n = round(t/obj.dt) + 1;
            track_error = obj.L2_error(obj.x_ref(:,n), x(1:2));
            
            energy_cost = obj.robot_model.power_elec(x,u) ;
            
            cost = track_error*obj.w_t + energy_cost*obj.w_e;
        end
        function cost = l_elec_rege(obj, x, u, t)
            n = round(t/obj.dt) + 1;
            track_error = obj.L2_error(obj.x_ref(:,n), x(1:2));
            
            energy_cost = obj.robot_model.power_elec(x,u)  ;
            p_rege = obj.robot_model.power_rege(x,u);
            cost = track_error*obj.w_t + energy_cost*obj.w_e - p_rege*obj.w_r;
        end
        function cost = l_elec_posi(obj, x, u, t)
            n = round(t/obj.dt) + 1;
            track_error = obj.L2_error(obj.x_ref(:,n), x(1:2));
            
            energy_cost = obj.robot_model.power_elec_posi(x,u) ;
            
            cost = track_error*obj.w_t + energy_cost*obj.w_e;
        end
        function cost = l_elec_posi_rege(obj, x, u, t)
            n = round(t/obj.dt) + 1;
            track_error = obj.L2_error(obj.x_ref(:,n), x(1:2));
            
            energy_cost = obj.robot_model.power_elec_posi(x,u)  ;
            p_rege = obj.robot_model.power_rege(x,u);
            cost = track_error*obj.w_t + energy_cost*obj.w_e - p_rege*obj.w_r;
        end
        function cost = l_f(obj, x)
            
            cost = obj.L2_error(obj.x_ref(:,end), x(1:2));
        end
        
    end
    
    methods (Static)
        function [x, xdot] = generate_simpletraj(amp, freq, t)
            x = -amp*cos(2*pi*freq.*t);
            xdot = amp*2*pi*freq*sin(2*pi*freq.*t);
        end
        
        function error = L2_error(xref, x)
            error = norm(xref - x,2);
        end
        
        function [ res ] = traj_features( model, x, u, dt, param)
            %TRAJ_FEATURES calculate all the features that can be used to evaluate and
            %characterise a trajectory and its performance
            Nx = size(x,2);
            Nu = size(u,2);
            stiffness = zeros(1,Nu);
            damping = zeros(1,Nu); % variable damping
            b = zeros(1,Nu);
            damping_ratio = zeros(1,Nu);
            p_load = zeros(1,Nu); % tau_load * v
            p_mech = zeros(1,Nu); % tau_m * v
            p1_mech = zeros(1,Nu);
            p2_mech = zeros(1,Nu);
            p_elec = zeros(1,Nu);
            p_elec2 = zeros(1,Nu);
            p1_elec = zeros(1,Nu);
            p2_elec = zeros(1,Nu);
            p1_elec2 = zeros(1,Nu);
            p2_elec2 = zeros(1,Nu);
            
            p_effort = zeros(1,Nu);
            %p1_diss = zeros(1,Nu);
            %p2_diss = zeros(1,Nu);
            p_rege = zeros(1,Nu);
            p_netelec = zeros(1,Nu);
            p_damp = zeros(1,Nu);
            res.tau_spring = zeros(1,Nu);
            p_link = zeros(1,Nu);
            p_fric = zeros(1,Nu);
            %cost = evaluate_trajectory_cost_fh();
            for i = 1:Nu
                stiffness(i) = model.stiffness(x(:,i));
                damping(i) = model.damping(x,u(:,i));
                b(i) = damping(i) + model.Df;
                p_fric(i) = model.Df*x(2,i)^2;
                damping_ratio(i) = model.damping_ratio(x(:,i),u(:,i));
                p_load(i) = model.power_load(x(:,i),u(:,i));
                [p_mech(i),p1_mech(i),p2_mech(i)] = model.power_mech(x(:,i),u(:,i));
                [p_elec(i),p1_elec(i),p2_elec(i)] = model.power_elec(x(:,i),u(:,i));
                [p_elec2(i),p1_elec2(i),p2_elec2(i)] = model.power_elec2(x(:,i),u(:,i));
                
                p_rege(i) = model.power_rege(x(:,i),u(:,i));
                p_netelec(i) = p1_elec(i) + p2_elec(i) - p_rege(i);
                res.tau_spring(i) = model.torque_spring(x(:,i));
                p_link(i) = res.tau_spring(i)*x(2,i);
                p_damp(i) = damping(i)*x(2,i)^2 ;
                p_effort(i) = (u(1,i)-x(3,i))^2 + (u(2,i)-x(4,i))^2;
            end
            
            p1_diss = p1_elec - p1_mech;
            p2_diss = p2_elec - p2_mech;
            res.E_diss = sum(p1_diss)*dt + sum(p2_diss)*dt;
            res.E_elec = sum(p1_elec)*dt+sum(p2_elec)*dt;
            res.E_elec2 = sum(p1_elec2)*dt+sum(p2_elec2)*dt;
            res.E_elec_posi = sum( max(0, p1_elec) )*dt + dt*sum( max(0, p2_elec) );
            
            res.E_mech = sum(p1_mech)*dt+sum(p2_mech)*dt;
            res.E_mech_posi = sum( max(0, p1_mech) )*dt + sum( max(0, p2_mech) )*dt;
            res.E_load = sum(p_load)*dt;
            res.E_load_posi = sum( max(0,p_load))*dt;
            
            res.E_rege = sum(p_rege)*dt;
            res.E_damp = sum(p_damp)*dt;
            res.E_link = sum(p_link)*dt ;
            res.E_link_posi = sum(max(0,p_link))*dt ;
            res.E_fric = sum(p_fric)*dt;
            res.E_effort = sum(p_effort)*dt;
            res.rege_ratio = res.E_rege/res.E_link;
            res.E_netelec = res.E_elec - sum(p_rege)*dt;
            res.E_netelec_posi = res.E_elec_posi - sum(p_rege)*dt;
            res.E_neteffort = res.E_effort - res.E_rege;
            res.E_netmech = res.E_mech - sum(p_rege)*dt;
            
            
            
            res.track_error = sum(sum( (param.x_ref - x(1:2,:) ).^2,1 ),2)*dt;
            
            res.peak_speed = max(x(2,:));
            
            res.elec_rate = res.E_elec/param.T;
            res.elec_posi_rate = res.E_elec_posi/param.T;
            res.effort_rate = res.E_effort/param.T;
            res.netelec_rate = res.E_netelec/param.T;
            res.netelec_posi_rate = res.E_netelec_posi/param.T;
            res.rege_rate = res.E_rege/param.T;
            
            
            res.stiffness = stiffness;
            res.damping = damping;
            res.b = b;
            res.damping_ratio = damping_ratio;
            res.p_outmech = p_load;
            res.p_mech = p_mech;
            res.p1_mech = p1_mech;
            res.p2_mech = p2_mech;
            res.p_elec = p_elec;
            res.p1_elec = p1_elec;
            res.p2_elec = p2_elec;
            res.p_rege = p_rege;
            res.p_netelec = p_netelec;
            res.p_link = p_link;
            res.p_effort = p_effort;
            
            res.p1_diss = p1_diss;
            res.p2_diss = p2_diss;
            res.p_damp = p_damp;
            
        end
        
        
    end
    
end

