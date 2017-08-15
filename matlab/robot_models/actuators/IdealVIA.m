classdef IdealVIA
    %IDEALVIA a ideal VIA model
    %
    
    properties
        %%%% - damping motor - %%%%
        %Id = 0.00184
        %gear_d = 20;
        %Kd = 0.0212;
        %Rd = 21.2;
        ratio_load = 1; % := R_l/R_m
        Rl
        %max_damping_db
        max_rege_damping
        max_damping
        u_max_regedamp
        
        max_stiffness
        
        umin  
        umax 
    end
    
    methods
        function obj = IdealVIA(varargin)
            if nargin > 0
                p = varargin{1};
                if isfield(p, 'umin'), obj.umin = p.umin; end
                if isfield(p, 'umax'), obj.umax = p.umax; end
                if isfield(p, 'ratio_load'),obj.ratio_load = p.ratio_load;end
                if isfield(p, 'max_damping'),obj.max_damping = p.max_damping;end
                if isfield(p, 'max_stiffness'), obj.max_stiffness=p.max_stiffness;end
            end
            
            obj.max_rege_damping = obj.max_damping/(1 + obj.ratio_load);
            obj.u_max_regedamp = 1/(1 + obj.ratio_load);
            
        end
        
        function tau = torque(obj, q, qdot, u1, u2, u3)
            k = obj.stiffness(u2);
            d = obj.damping(u3);
            tau = k*(u1 - q) - d*qdot;
        end
        function tau = torque_k(obj, q, u1, u2)
            k = obj.stiffness(u2);
            
            tau = k*(u1 - q) ;
        end
        function k = stiffness(obj, u2)
            k = u2*obj.max_stiffness + 10;
        end
        function d = damping(obj, u3)
            %d = u3*obj.max_damping;
            
            r = obj.ratio_load;
            if u3 <= obj.u_max_regedamp
                D1 = u3/obj.u_max_regedamp;
                D2 = 0;
            elseif u3 <= 1
                D1 = 1;
                D2 = ( u3 - obj.u_max_regedamp )/( 1 - obj.u_max_regedamp );                
            end
            
            d = obj.max_rege_damping*D1 + obj.max_rege_damping*D2*r;
            
        end
        
        function tau_d = torque_damp(obj, qdot, u3)
            tau_d = obj.damping(u3)*qdot;
        end
        
        function power = power_rege(obj, qdot, u3)
            
            r = obj.ratio_load;
            alpha = r/(1+r);
            if u3 <= obj.u_max_regedamp
                D1 = u3/obj.u_max_regedamp;
                D2 = 0;
            elseif u3 <= 1
                D1 = 1;
                D2 = ( u3 - obj.u_max_regedamp )/( 1 - obj.u_max_regedamp );  
            elseif u3 > 1
                D1 = 1;
                D2 = 1;
            end
            
            power = obj.max_rege_damping*alpha*qdot^2*D1 - ...
                obj.max_rege_damping*alpha*qdot^2*D2;
            
        end
    end
    
    
end

