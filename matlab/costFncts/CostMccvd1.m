classdef CostMccvd1 < CostFunction
    %COSTMCCVD1 Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        
    end
    
    methods
        % class constructor. need to call superclass constructor
        function obj = CostMccvd1()
            obj@CostFunction();
        end
    end
    
    methods (Static)

        function [ l, l_x, l_xx, l_u, l_uu, l_ux ] = j_reach_netmech(robot_model,x,u,t,p )
            %J_REACH_MW_MCC1DOF_MD cost with energy cost measured by mechanical work.
            %Fot 1dof MACCEPA-VD with motor dynamics.
            %   x : column vector, 8 elements
            %   u : command input
            %   t : time
            %   p : parameters
            %   by Fan WU 29 Jun 2016. fan.wu@kcl.ac.uk
            
            
            fl = @(x,u,t) CostMccvd1.l_reach_netmech(robot_model,x,u,t,p);
            l = fl(x,u,t);
            
            
            if nargout>1
                if(~ isfield(p,'fd')), p.fd = 1;end
                
                if p.fd == 0
                    % analytical derivatives
                    % todo: this does not include motor dynamics now
                    l_x = [2*(x(1) - p.target);0] ;
                    l_u = 2*p.epsilon*u ;
                    l_xx = [ 2 0;0 0];
                    l_uu = [2*p.epsilon 0 0;0 2*p.epsilon 0;0 0 2*p.epsilon] ;
                    l_ux = zeros(3,2);
                elseif p.fd == 1
                    
                    % finite difference
                    flJ=@(x,u,t)J_cost_fd ( fl, x, u, t );
                    [l_x ,l_u      ] = flJ ( x, u, t );
                    flH =@(x,u,t)H_cost_fd  ( flJ, x, u, t );
                    [l_xx,l_uu,l_ux] = flH  ( x, u, t );
                else
                    error('missing or wrong value of p.fd')
                end
                
            end
            
        end
        
        function [ c ] = l_reach_netmech(robot_model, x, u, t, p )
            %L_REACH_MW_MCC1DOF_MD running cost of 1dof maccepa-vd with motor dynamics
            %   Detailed explanation goes here
            c1 = (x(1,:) - p.target).^2;
            
            if isnan(t) || t==p.T
                %final cost
                c = c1 * p.dt ;
            else
                % running cost
                mechpower = robot_model.mechpower(x ,u);
                damping = robot_model.damping(u(3));
                power = mechpower - p.alpha*damping*(x(2)^2);
                if(~isfield(p,'w0')), w0 = 1;else, w0=p.w0; end
                c =  c1*w0 + sum(u.^2,1) * p.epsilon + power*p.w;
            end
            
        end
        
        
    end
    
end

