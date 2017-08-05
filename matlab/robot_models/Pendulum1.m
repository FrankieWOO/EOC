classdef Pendulum1
    %PENDULUM1 Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        m = 1
        l = 1
        g = 0
        d = 0.01
        I
        actuator
        has_gravity = false
        
        umin = [-pi/2; 0; 0];
        umax = [ pi/2; 1; 1];
        
        dimX = 2
        dimU = 3
    end
    
    methods
        function obj = Pendulum1(varargin)
            
            if nargin > 0
                p = varargin{1};
                if isfield(p, 'has_gravity'), obj.has_gravity = p.has_gravity;end
                if obj.has_gravity == true, obj.g = 9.8; end
                if isfield(p, 'umin'), obj.umin = p.umin;end
                if isfield(p, 'umax'), obj.umax = p.umax;end
                if isfield(p, 'dimU'), obj.dimU = p.dimU;end
            end
            if nargin > 1
                p_act = varargin{2};
                p_act.umin = obj.umin;
                p_act.umax = obj.umax;
                obj.actuator = IdealVIA(p_act);
            end
            obj.I = obj.m*(obj.l^2);
        end
        function [xdot, xdot_x, xdot_u] = dynamics(obj, x, u)
            qddot = obj.accel(x,u);
            xdot = [x(2); qddot];
            
            if nargout > 1
                %xdot_x = [0 1;
                %         -obj.stiffness(u) -obj.damping(u)];
                f = @(x)obj.accel(x,u);
                 daccdx = get_jacobian_fd(f,x);
                 if iscolumn(daccdx), daccdx = daccdx';end
                 % Compute derivative of acceleration w.r.t. u
                 %daccdu = zeros(1,model.dimU);
                 f = @(u)obj.accel(x,u);
                 daccdu = get_jacobian_fd(f,u);
                 if iscolumn(daccdu), daccdu = daccdu'; end
                 xdot_x = [0, 1;
                             daccdx] ;
                             
           
                 xdot_u = [zeros(1,3);
                         daccdu]; 
                         
                    
            end
        end
        
        function accel = accel(obj, x, u)
            torque = obj.torque(x, u);
            accel = (torque - obj.m*obj.g*obj.l*sin(x(1)) - obj.d*x(2))/obj.I;
        end
        
        function accel = accel2(obj, x, u1, u2, u3)
            % dynamic function for fixing u1, u2, or u3
            u = [u1;u2;u3];
            torque = obj.torque(x, u);
            accel = (torque - obj.m*obj.g*obj.l*sin(x(1)) - obj.d*x(2))/obj.I;
        end
        
        function tau = torque(obj, x, u)
            % q, qdot, u1, u2, u3
            tau = obj.actuator.torque(x(1),x(2),u(1),u(2),u(3));
        end
        function k = stiffness(obj, u)
            k = obj.actuator.stiffness(u(2));
        end
        function d = damping(obj, u)
            d = obj.actuator.damping(u(3));
        end
        
        function power = power_rege(obj,x ,u)
            power = obj.actuator.power_rege(x(2),u(3));
        end
    end
    
end

