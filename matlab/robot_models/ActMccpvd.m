classdef ActMccpvd
    %ACTMCCPVD Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        B = 0.036
        C = 0.135
        A0 = 0.099 % C - B
        Ks = 231 % spring constant
        r = 0.015
        gear = 3/4 % gear ratio between servo1 and link axis
        damping_range = [0, 0.00848]; % 0.00848
    end
    
    methods
        function obj = ActMccpvd()
            
        end
        
        %variable damping
        function d = damping(obj,duty_circle)
            % duty_circle: 0-1
            % linear on duty-circle
            d = obj.damping_range(1) + duty_circle.*(obj.damping_range(2)-obj.damping_range(1));
        end
        
        function torque_damp = torque_damping(obj,qdot, dc)
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
            
            A = sqrt( obj.C^2 + obj.B^2 - 2*obj.B*obj.C.*cos( theta1/obj.gear1-q ) );
            phi = theta1/obj.gear - q;
            k=  obj.Ks*obj.B*obj.C*cos(phi)*( 1 + (obj.r*theta2 - obj.A0)/A ) - ...
                obj.Ks*(obj.B*obj.C*sin(phi))^2*( obj.r*theta2 - obj.A0 )/A^1.5 ;
        end
        
    end
    
end

