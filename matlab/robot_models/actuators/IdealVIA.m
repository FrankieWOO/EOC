classdef IdealVIA
    %IDEALVIA a ideal VIA model
    %
    
    properties
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
        function self = IdealVIA()
            
        end
        
        function tau = torque(q,qdot,u1,u2,u3)
            tau = u2*(u1 - q) - u3*qdot;
        end
    end
    
    
end

