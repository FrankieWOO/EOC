classdef pendulum1_reach
    %PENDULUM1_REACH Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        f
        j
        model % robot model
        
        target
        w_t
        w_e
        w_tf
        w_r
        dt
        
    end
    
    methods
        function obj = pendulum1_reach(p)
            obj.target = p.target;
            obj.w_t = p.w_t;
            obj.w_e = p.w_e;
            
            obj.w_tf = p.w_tf;
            
            obj.w_r = p.w_r;
            obj.dt = p.dt;
        end
        
        function [l, l_x, l_xx, l_u, l_uu, l_ux] = j_reach(obj, x, u, t)
            if (isnan(u))
                % final cost
                fl = @(x) obj.costf_reach(x);
                l = fl(x);
                if nargout>1
                    flJ = @(x) get_jacobian_fd(fl, x);
                    l_x = flJ(x);
                    l_xx = get_hessian_fd(flJ,x);
                end
            else
                fl = @(x,u,t) obj.costr_effort(x,u);
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
        function [l, l_x, l_xx, l_u, l_uu, l_ux] = j_fastreach(obj, x, u, t)
            if (isnan(u))
                % final cost
                fl = @(x) obj.costf(x);
                l = fl(x);
                if nargout>1
                    flJ = @(x) get_jacobian_fd(fl, x);
                    l_x = flJ(x);
                    l_xx = get_hessian_fd(flJ,x);
                end
            else
                fl = @(x,u,t) obj.costr_effort(x,u);
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
        function cost = costr_effort(obj, x, u)
            c1 = (x(1) - obj.target).^2;
            %ce = (u(1)-x(3))^2 + (u(2)-x(4))^2;
            ce = (u(1) - obj.target)^2 + u(2)^2;
            
            cost = c1*obj.w_t + ce*obj.w_e;
        end
        function cost = costr_effort_rege(obj, x, u)
            c1 = (x(1) - obj.target).^2;
            %ce = (u(1)-x(3))^2 + (u(2)-x(4))^2;
            ce = (u(1) - obj.target)^2 + u(2)^2;
            p_rege = obj.robot_model.power_rege(x,u);
            cost = c1*obj.w_t + ce*obj.w_e - p_rege*obj.w_r;
        end
        function cost = costf(obj, x)
            cost = (x(1) - obj.target).^2 * obj.dt * obj.w_tf;
        end
        function cost = costf_reach(obj, x)
            % penalise both position and speed errors at T
            errorp = abs( x(1) - obj.target);
            errorv = abs(x(2));
            %errora = abs( x(3) - x(1) );
            if errorp <= 0.001, errorp = 0;end
            if errorv <= 0.001, errorv = 0;end
            %if errora <= 0.01, errora = 0;end
            %error = errorp + errorv + errora;
            cost = (errorp^2 + errorv^2 )*obj.w_tf;
        end
    end
    
end

