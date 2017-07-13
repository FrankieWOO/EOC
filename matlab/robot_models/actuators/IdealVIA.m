classdef IdealVIA
    %IDEALVIA a ideal VIA model
    %
    
    properties
        %%%% - damping motor - %%%%
        %Id = 0.00184
        %gear_d = 20;
        %Kd = 0.0212;
        %Rd = 21.2;
        ratio_load = 1;
        Rl
        %max_damping_db
        max_damping
        
        umin  
        umax 
    end
    
    methods
        function obj = IdealVIA(varargin)
            if nargin > 0
                p = varargin{1};
                if isfield(p, 'umin'), obj.umin = p.umin; end
                if isfield(p, 'umax'), obj.umax = p.umax; end
                if isfield(p, 'ratio_load'), obj.ratio_load=p.ratio_load;end
            end
        end
        
        function tau = torque(obj, q, qdot, u1, u2, u3)
            tau = u2*(u1 - q) - u3*qdot;
        end
        
        function d = damping(obj, u3)
            d = u3;
        end
        
        function tau_d = torque_damp(obj, qdot, u3)
            tau_d = obj.damping(u3)*qdot;
        end
        
        function power = power_rege(obj, qdot, u3)
            power = obj.torque_damp(qdot, u3)*qdot*obj.ratio_load/(1+obj.ratio_load);
        end
    end
    
    
end

