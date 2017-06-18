classdef mccpvd1_reach
    %MCCPVD1_REACH robot specific task definition
    %   Define robot-task specific cost functions
    % costf: final cost function
    % costr: running cost function
    
    properties
        robot_model
        target
        w
        w0 = 1 % task term weight
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
            %self.alpha = p.alpha;
            self.epsilon = p.epsilon;
            self.dt = p.dt;
            self.T = p.T;
        end
        function [l, l_x, l_xx, l_u, l_uu, l_ux] = j_outmech(self,x,u,t)
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
                fl = @(x,u,t) self.costr_outmech(x,u);
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
        function [l, l_x, l_xx, l_u, l_uu, l_ux] = j_elec(self,x,u,t)
            % Energy Cost := sum[ positive elec power]
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
                fl = @(x,u,t) self.costr_elec(x,u);
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
        function [l, l_x, l_xx, l_u, l_uu, l_ux] = j_netelec(self,x,u,t)
            % Energy Cost := sum[ positive elec power]
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
                fl = @(x,u,t) self.costr_netelec(x,u);
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
        
        function [l, l_x, l_xx, l_u, l_uu, l_ux] = j_net2(self,x,u,t)
            %net total energy cost
            if (isnan(u))
                % final cost
                fl = @(x) self.costf3(x);
                l = fl(x);
                if nargout>1
                    flJ = @(x) get_jacobian_fd(fl, x);
                    l_x = flJ(x);
                    l_xx = get_hessian_fd(flJ,x);
                end
            else
                fl = @(x,u,t) self.costr_net2(x,u);
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
               
        function [l, l_x, l_xx, l_u, l_uu, l_ux] = j_net3(self,x,u,t)
            %net total energy cost
            if (isnan(u))
                % final cost
                fl = @(x) self.costf3(x);
                l = fl(x);
                if nargout>1
                    flJ = @(x) get_jacobian_fd(fl, x);
                    l_x = flJ(x);
                    l_xx = get_hessian_fd(flJ,x);
                end
            else
                fl = @(x,u,t) self.costr_net3(x,u,t);
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
        function c = costr_outmech(self, x, u)
            
            c1 = (x(1) - self.target).^2;
            
            % running cost
            [p,p1,p2] = self.robot_model.output_mechpower(x ,u);
            
            power = max(0,p1) + max(0,p2) ;
            c =  c1*self.w0 + sum(u.^2,1) * self.epsilon + power*self.w;
        end
        function c = costr_noutmech(self, x, u)
            
            c1 = (x(1) - self.target).^2;
            
            % running cost
            [p,p1,p2] = self.robot_model.output_mechpower(x ,u);
            %damping = self.robot_model.actuator.damping(u(3));
            
            p_charge = self.robot_model.power_charge(x,u);
            power = max(0,p1) + max(0,p2) - p_charge;
            c =  c1*self.w0 + sum(u.^2,1) * self.epsilon + power*self.w;
        end
        
        function c = costr_elec(self, x, u)
            c1 = (x(1) - self.target).^2;
            
            [p_total, p1_elec, p2_elec] = self.robot_model.power_elec(x,u);
            p1_elec = max(0,p1_elec);
            p2_elec = max(0,p2_elec);
            power = p1_elec + p2_elec ;
            c =  c1*self.w0 + sum(u.^2,1) * self.epsilon + power*self.w;

        end
        function c = costr_netelec(self, x, u)
            c1 = (x(1) - self.target).^2;
            
            [p_total, p1_elec, p2_elec] = self.robot_model.power_elec(x,u);
            p1_elec = max(0,p1_elec);
            p2_elec = max(0,p2_elec);
            p_charge = self.robot_model.power_charge(x,u);
            power = p1_elec + p2_elec - p_charge;
            
            c =  c1*self.w0 + sum(u.^2,1) * self.epsilon + power*self.w;
            
        end
        
        function c = costr_net(self, x, u)
            c1 = (x(1) - self.target).^2;
            
            [p_total, p1_elec, p2_elec] = self.robot_model.power_elec(x,u);
            damping = self.robot_model.actuator.damping(u(3));
            p1_elec = max(0,p1_elec);
            p2_elec = max(0,p2_elec);
            power = p1_elec + p2_elec - self.alpha*damping*(x(2)^2);
            c =  c1*self.w0 + sum(u.^2,1) * self.epsilon + power*self.w;
        end
        
        function c = costr_net2(self, x, u)
            p_total = self.robot_model.total_power(x,u);
            damping = self.robot_model.actuator.damping(u(3));
            power = p_total - self.alpha*damping*(x(2)^2);
            c = sum(u.^2,1)*self.epsilon + power*self.w; 
        end
        
        function c = costr_net3(self, x, u, t)
            N = self.T/self.dt + 1; % N: number of state sequence
            if t/self.dt+1 > N-10
                c1 = 50*(x(1) - self.target).^2;
            else
                c1 = 0;
            end
            p_total = self.robot_model.total_power(x,u);
            damping = self.robot_model.actuator.damping(u(3));
            power = p_total - self.alpha*damping*(x(2)^2);
            c = c1*self.w0 + sum(u.^2,1)*self.epsilon + power*self.w; 
        end
        
        function c= costf(self, x)
            c = (x(1) - self.target).^2 * self.dt * self.w0;
        end
        
        function c = costf2(self, x)
            % penalise position error at T without multiply by dt
            c = (x(1) - self.target)^2 * self.w0 ;
        end
        function c = costf3(self, x)
            % penalise both position and speed errors at T
            c = ((x(1) - self.target)^2 + x(2)^2)*self.w0 ;
        end

    end
end

