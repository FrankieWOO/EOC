classdef ActMccpvd
    %ACTMCCPVD Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        B = 0.036
        C = 0.135
        A0 = 0.099 % C - B
        Ks = 394 %231 % spring constant
        r = 0.015
        gear = 1; %3/4 % gear ratio between servo1 and link axis
        %damping_range = [0, 0.00848]; % 0.00848
        damping_range = [0, 0.00848]; % 0.00848
        
        J1 = 0.0099;
        K1 = 0.3811; % torque constant
        D1 = 0.2009;
        R1 = 0.8222;
        J2 = 0.0099;
        K2 = 0.3811;
        D2 = 0.2009;
        R2 = 0.8222;
        
        %%%% - damping motor - %%%%
        Id = 0.00184
        gear_d = 20;
        Kd = 0.0212;
        Rd = 21.2;
        ratio_load = 0.5;
        Rl
        max_damping_db
        max_damping
    end
    
    methods
        function obj = ActMccpvd(varargin)
            if nargin > 0
                param = varargin{1};
                if isfield(param,'ratio_load'), obj.ratio_load = param.ratio_load; end
            end
            obj.Rl = obj.Rd*obj.ratio_load;
            obj.max_damping_db = obj.Kd^2 * obj.gear_d^2/obj.Rd ;
            obj.max_damping = obj.Kd^2 * obj.gear_d^2/(obj.Rd+obj.Rl);
        end
        
        %variable damping
        function d = damping(obj,duty_circle)
            % duty_circle: 0-1
            % 
            % Fan: modified 18/06/17
            d = obj.max_damping*obj.transm(duty_circle);
            %d = obj.damping_range(1) + duty_circle.*(obj.damping_range(2)-obj.damping_range(1));
        end
        
        function tr = transm(obj, u)
            % u :- u3
            tr = 1*u;
        end
        
        function p = p_damp_charge(obj, qdot, u)
            p = obj.p_damp_inputelec(qdot,u)* obj.ratio_load/(1+obj.ratio_load);
        end
        function p = p_damp_inputelec(obj, qdot, u)
            p = obj.gear_d^2*obj.Kd^2*qdot^2*obj.transm(u)^2/(obj.Rd+obj.Rl);
        end
        function p = p_damp_inputmech(obj, qdot, u)
            p = obj.gear_d^2*obj.Kd^2*qdot^2*obj.transm(u)/(obj.Rd+obj.Rl);
        end
        function torque_damp = torque_damping(obj,qdot, dc)
            % damping torque
            % note size(x,2) == size(u,2)
            torque_damp = qdot.*obj.damping(dc);
        end
        
        
        
        function torque = torque(obj, q, qdot, m1, m2, dc)
            torque = obj.torque_spring(q, m1, m2) - obj.torque_damping(qdot,dc);
        end
        
        function torque = torque_spring(obj, q, m1, m2)
            %sprdis = model.spring_displacement(x,u);
            %spring_force = sprdis*model.spring_constant;
            phi = m1/obj.gear - q;
            A = sqrt( obj.C^2 + obj.B^2 - 2*(obj.B*obj.C).*cos( phi ));
            
            torque = obj.Ks*(obj.B*obj.C).*sin(phi).*...
                (1+ (obj.r*m2 - obj.A0)./A );
        end
        
        
        function sprlength = spring_displacement(obj, q, theta1, theta2)
            
            A = sqrt(obj.C^2 + obj.B^2 - 2*obj.B*obj.C*cos(theta1/obj.gear-q));
            sprlength = A + obj.r*theta2 - obj.A0; 
        end
        
        function k = stiffness(obj, q, theta1, theta2)
            
            A = sqrt( obj.C^2 + obj.B^2 - 2*obj.B*obj.C.*cos( theta1/obj.gear-q ) );
            phi = theta1/obj.gear - q;
            k=  obj.Ks*obj.B*obj.C*cos(phi)*( 1 + (obj.r*theta2 - obj.A0)/A ) - ...
                obj.Ks*(obj.B*obj.C*sin(phi))^2*( obj.r*theta2 - obj.A0 )/A^1.5 ;
        end
        
    end
    
end

