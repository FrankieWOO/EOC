classdef CostFunction
    %COSTFUNCTION Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        
    end
    
    methods
        function obj = CostFunction()
            
        end
    end
    
    methods (Static)
        function [ l, l_x, l_xx, l_u, l_uu, l_ux ] = j_reach_squ(x,u,t,p )
            %J_REACH_MW_MCC1DOF_MD cost with energy cost measured by squared control inputs.
            %Fot 1dof MACCEPA-VD with motor dynamics.
            %   x : column vector, 8 elements
            %   u : command input
            %   t : time
            %   p : parameters
            %   by Fan WU 29 Jun 2016. fan.wu@kcl.ac.uk
            
            
            fl = @(x,u,t) CostMccvd1.l_reach_squ(x,u,t,p);
            l = fl(x,u,t);
            
            
            if nargout>1
                if(~ isfield(p,'fd')), p.fd = 1;end
                
                if p.fd == 0
                    % analytical derivatives
                    % todo: this does not include motor dynamics now
                    l_x = [2*(x(1) - p.x_target);0] ;
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
        
        function [ c ] = l_reach_squ( x, u, t, p )
            %L_REACH_MW_MCC1DOF_MD running cost of 1dof maccepa-vd with motor dynamics
            %   Detailed explanation goes here
            c1 = (x(1,:) - p.x_target).^2;
            
            if isnan(t) || t==p.T
                %final cost
                c = c1 * p.dt ;
            else
                % running cost
                if( size(u,2)~=1 && size(u,1)~=1 ), error('u should be a vector');end
                power = sum(u.^2);
                if(~isfield(p,'w0')), w0 = 1;else, w0=p.w0; end
                c =  c1*w0 + sum(u.^2,1) * p.epsilon + power*p.w;
            end
            
        end
        
        
        
        function cost = squared_distance( x,target)
            cost = (x - target).^2;
        end
        function cost = l_error_1dof(x,target)
            cost = OptTask.squared_distance( x(1), target );
        end
        function cost = l_error_2dof(x,target)
            cost = OptTask.squared_distance( x([1,3]), target);
        end
        function cost = l_error_3dof(x,target)
            cost = OptTask.squared_distance( x([1,3,5]), target);
        end
        function cost = control_effort(u)
            cost = sum(u.^2);
        end
        
    end
    
end

