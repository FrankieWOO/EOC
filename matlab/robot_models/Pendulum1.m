classdef Pendulum1
    %PENDULUM1 Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        m = 1
        l = 1
        g = 0
        d = 0.1
        I
        actuator
        has_gravity = false
    end
    
    methods
        function obj = Pendulum1(varargin)
            obj.actuator = IdealVIA();
            if nargin > 0
                p = varargin{1};
                if isfield(p, 'has_gravity'), obj.has_gravity = p.has_gravity;end
                if obj.has_gravity == true, obj.g = 9.8;end
            end
            obj.I = obj.m*(obj.l^2);
        end
        
        function accel = dynamics(obj, x, u)
            torque = obj.torque(x, u);
            accel = (torque - obj.m*obj.g*obj.l*sin(x(1)) - obj.d*x(2))/obj.I;
        end
        
        function accel = dynamics2(obj, x, u1, u2, u3)
            % dynamic function for fixing u1, u2, or u3
            u = [u1;u2;u3];
            torque = obj.torque(x, u);
            accel = (torque - obj.m*obj.g*obj.l*sin(x(1)) - obj.d*x(2))/obj.I;
        end
        
        function tau = torque(obj, x, u)
            % q, qdot, u1, u2, u3
            tau = obj.actuator.torque(x(1),x(2),u(1),u(2),u(3));
        end
    end
    
end

