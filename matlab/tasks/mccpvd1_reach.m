classdef mccpvd1_reach
    %MCCPVD1_REACH robot specific task definition
    %   Define robot-task specific cost functions
    % costf: final cost function
    % costr: running cost function
    
    properties
        robot_model
        target
        w
        w0 = 1
        alpha % energy regeneration rate
        epsilon = 0
        fd = 1
        dt
        T % final time
    end
    
    methods
        function self = mccpvd1_reach(robot_model, p)
            self.robot_model = robot_model;
            self.target = p.target;
            self.w = p.w;
            self.alpha = p.alpha;
            self.epsilon = p.epsilon;
            self.dt = p.dt;
            self.T = p.T;
        end
        
        function [l, l_x, l_xx, l_u, l_uu, l_ux] = j_noutmech(self,x,u,t)
            % net output mechanical power
            if (isnan(u))
                % final cost
                fl = @(x) self.costf(x);
                l = fl(x);
                if nargout>1
                    flJ = @(x) get_jacobian_fd(fl, x);
                    l_x = flJ(x);
                    l_xx = get_hessian_fd(flJ,x);
                end
            else
                fl = @(x,u,t) self.costr_noutmech(x,u);
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
        
        
        
        function c = costr_noutmech(self, x, u)
            
            c1 = (x(1) - self.target).^2;
            
            % running cost
            mechpower = self.robot_model.output_mechpower(x ,u);
            damping = self.robot_model.damping(u(3));
            power = mechpower - self.alpha*damping*(x(2)^2);
            
            c =  c1*self.w0 + sum(u.^2,1) * self.epsilon + power*self.w;
        end
        
        function [l, l_x, l_xx, l_u, l_uu, l_ux] = j_net(self,x,u,t)
            %net total energy cost
            if (isnan(u))
                % final cost
                fl = @(x) self.costf(x);
                l = fl(x);
                if nargout>1
                    flJ = @(x) get_jacobian_fd(fl, x);
                    l_x = flJ(x);
                    l_xx = get_hessian_fd(flJ,x);
                end
            else
                fl = @(x,u,t) self.costr_net(x,u);
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
        
        function c= costf(self, x)
            c = (x(1) - self.target).^2 * self.dt * self.w0;
        end
    end
end

