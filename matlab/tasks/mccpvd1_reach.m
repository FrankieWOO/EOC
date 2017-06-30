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
        
        function [l, l_x, l_xx, l_u, l_uu, l_ux] = j_effort(self,x,u,t)
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
                fl = @(x,u,t) self.costr_effort(x,u);
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
        
        function [l, l_x, l_xx, l_u, l_uu, l_ux] = j_netmech(self,x,u,t)
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
                fl = @(x,u,t) self.costr_netmech(x,u);
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
        function [l, l_x, l_xx, l_u, l_uu, l_ux] = j_mech(self,x,u,t)
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
                fl = @(x,u,t) self.costr_mech(x,u);
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
        
        function c = costr_mech(self, x, u)
            
            c1 = (x(1) - self.target).^2;
            
            % running cost
            [p,p1,p2] = self.robot_model.power_mech(x ,u);
            
            power = max(0,p1) + max(0,p2) ;
            c =  c1*self.w0 + sum(u.^2,1) * self.epsilon + power*self.w;
        end
        function c = costr_netmech(self, x, u)
            
            c1 = (x(1) - self.target).^2;
            
            % running cost
            [p,p1,p2] = self.robot_model.power_mech(x ,u);
            p_rege = self.robot_model.power_charge(x,u);
            power = max(0,p1) + max(0,p2) - p_rege ;
            c =  c1*self.w0 + sum(u.^2,1) * self.epsilon + power*self.w;
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
        function c = costr_effort(self, x,u)
            c1 = (x(1) - self.target).^2;
            ce = (u(1)-x(3))^2 + (u(2)-x(4))^2;
            c = c1*self.w0 + sum(u.^2,1) * self.epsilon + ce*self.w;
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
            % no error/accuracy term
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
        
        function [ res ] = traj_features(self, x, u, t, dt)
            %TRAJ_FEATURES calculate all the features that can be used to evaluate and
            %characterise a trajectory and its performance
            
            stiffness = zeros(size(t));
            damping = zeros(size(t)); % variable damping
            b = zeros(size(t));
            damping_ratio = zeros(size(t));
            p_outmech = zeros(size(t)); % tau_load * v
            p_mech = zeros(size(t)); % tau_m * v
            p1_mech = zeros(size(t));
            p2_mech = zeros(size(t));
            p_elec = zeros(size(t));
            p_elec2 = zeros(size(t));
            p1_elec = zeros(size(t));
            p2_elec = zeros(size(t));
            p1_elec2 = zeros(size(t));
            p2_elec2 = zeros(size(t));
            
            p1_diss = zeros(size(t));
            p2_diss = zeros(size(t));
            p_rege = zeros(size(t));
            p_netelec = zeros(size(t));
            p_damp = zeros(size(t));
            res.tau_spring = zeros(size(t));
            %cost = evaluate_trajectory_cost_fh();
            for i = 1:length(t)-1
                stiffness(i) = self.robot_model.stiffness(x(:,i));
                damping(i) = self.robot_model.damping(x,u(:,i));
                b(i) = damping(i) + self.robot_model.Df;
                damping_ratio(i) = self.robot_model.damping_ratio(x(:,i),u(:,i));
                p_outmech(i) = self.robot_model.output_mechpower(x(:,i),u(:,i));
                [p_mech(i),p1_mech(i),p2_mech(i)] = self.robot_model.power_mech(x(:,i),u(:,i));
                [p_elec(i),p1_elec(i),p2_elec(i)] = self.robot_model.power_elec(x(:,i),u(:,i));
                [p_elec2(i),p1_elec2(i),p2_elec2(i)] = self.robot_model.power_elec2(x(:,i),u(:,i));
                
                p_rege(i) = self.robot_model.power_charge(x(:,i),u(:,i));
                p_netelec(i) = max(0,p1_elec(i))+max(0,p2_elec(i))-p_rege(i);
                res.tau_spring(i) = self.robot_model.torque_spring(x(:,i));
                p_damp(i) = damping(i)*x(2,i)^2 ;
            end
            
            p1_diss = p1_elec - p1_mech;
            p2_diss = p2_elec - p2_mech;
            res.E_diss = sum(p1_diss)*dt + sum(p2_diss)*dt;
            res.E_elec = sum(p1_elec)*dt+sum(p2_elec)*dt;
            res.E_elec2 = sum(p1_elec2)*dt+sum(p2_elec2)*dt;
            res.E_elec_posi = sum( max(0, p1_elec) )*dt + dt*sum( max(0, p2_elec) );
            
            res.E_mech = sum(p1_mech)*dt+sum(p2_mech)*dt;
            res.E_mech_posi = sum( max(0, p1_mech) )*dt + sum( max(0, p2_mech) )*dt;
            res.E_load = sum(p_outmech)*dt;
            res.E_load_posi = sum( max(0,p_outmech))*dt;
            res.E_rege = sum(p_rege)*dt;
            res.E_damp = sum(p_damp)*dt;
            res.E_netelec = res.E_elec_posi - sum(p_rege)*dt;
            res.E_netmech = res.E_mech_posi - sum(p_rege)*dt;
            
            
            res.cost_accuracy = sum((self.target-x(1,:)).^2 )*dt;
            res.peak_speed = max(x(2,:));
            
            res.stiffness = stiffness;
            res.damping = damping;
            res.b = b;
            res.damping_ratio = damping_ratio;
            res.p_outmech = p_outmech;
            res.p_mech = p_mech;
            res.p1_mech = p1_mech;
            res.p2_mech = p2_mech;
            res.p_elec = p_elec;
            res.p1_elec = p1_elec;
            res.p2_elec = p2_elec;
            res.p_rege = p_rege;
            res.p_netelec = p_netelec;
            
            
            res.p1_diss = p1_diss;
            res.p2_diss = p2_diss;
            res.p_damp = p_damp;
            
        end
        
        
    end
end

