classdef IdealVIA
    %IDEALVIA a ideal VIA model
    %
    
    properties
    end
    
    methods
        function self = IdealVIA()
            
        end
        
        function tau = torque(q,qdot,u1,u2,u3)
            tau = u2*(u1 - q) - u3*qdot;
        end
    end
    
    
end

