classdef Maccepavd1dof
    %   MACCEPA-VD
    %   Detailed explanation goes here
    %   1DoF MACCEPAVD
    %   x: q ; dq ; theta1 ; dtheta1;ddtheta1;theta2;  dtheta2;  ddtheta2;
    %   
    %  
    
    properties
        robot_type = 'Maccepavd1DOF';
        
        cdt = 0.02 % controller time step
        
        dimQ = 1% dimension of joints
        dimX = 8% dimension of states
        dimU = 3% dimension of controls
        
        inertia = 0.0061% inertia
        
        %%%% physical design
        % physical joint constriants
        % note that the physical constriant of joint is the rechable point
        % not the controllable point
        qmax = 135*pi/180; %
        qmin = -135*pi/180; %
        % gear ratio from theta1 to q
        gear1 = 1; %1.4; 
        
        spring_constant = 323;
        link_length = 0.2950;% link length
        lever_length = 0.03; %B
        pin_displacement = 0.18; %C
        A0
        drum_radius = 0.015;
        gravity_constant = 0;
         % frictions
        viscous_friction = 0.013 %0.0025;
        coulomb_friction = 0;
        
       
        %variable damping range
        % 110255 : 4.2400e-04
        damping_range = [0, 0.1070];
        rege_ratio = 0;
        
        
        %%%% ---- physical design
        
        %%%% control input limits
        umax = [pi/3; pi; 1] ;
        umin = [-pi/3; 0; 0] ;
        %%%%
        
       
        
        %%%% function handlers
        fnMotorDyn
        dynx
        dynu
        %%%%
        
        %%%% symbolic
        symDyn
        symDynx
        symDynu
        
        
        %%%%
    end
    
    properties (Access = private)
        dynxfull
        dynufull
    end
    
    methods
        function model = Maccepavd1dof()
            
            model.A0 = model.pin_displacement - model.lever_length; %A0 = C-B
            
            try
            %servo_param.p=[14000;1150;38;14000;1150;38];
            servo_param = load('maccepa_servomotor_model.mat');
            model.fnMotorDyn =@(x,u)fn_dynamics_servomotor_3rd_order_filter(x,u,servo_param.p); 
            catch
                fprintf(1,'Fit servomotor model first please.')
            end
            
            %%%% setup symbolic dynamics and compute jacobian functions
            xv=sym('x',[8 1]);
            uv=sym('u',[3 1]);
            model.symDyn = symfun(model.dynamics(xv,uv),[xv;uv]);
            model.symDynx = jacobian(model.symDyn,xv);
            model.symDynu = jacobian(model.symDyn,uv);
            model.dynxfull = matlabFunction(model.symDynx);
            model.dynufull = matlabFunction(model.symDynu);
            model.dynx =@(x,u)model.dynxfull (x(1),x(2),x(3),x(4),x(5),x(6),x(7),x(8),u(1),u(2),u(3));
            model.dynu =@(x,u) model.dynufull(x(1),x(2),x(3),x(4),x(5),x(6),x(7),x(8),u(1),u(2),u(3));
            %model.dynx = matlabFunction(model.dynx);
            %model.dynu = matlabFunction(model.dynu);
            %%%%
        end
        
        function [xdot]= dynamics(model, x, u)
            qddot = model.cmptAcc(x,u);
            qdot = x(2,:);
            xdot1 = [qdot; qddot] ;

            % motor dynamics
            xdot2 = model.fnMotorDyn(x(3:end),u(1:2));
            
            xdot  = [xdot1;
                    xdot2];
            % current motor positions
            %m     = [x(3);x(6)] ; 
            %if nargout > 1
            % Compute xdot_x, xdot_u using finite differences
                %delta = 1e-6;
                %dimX = size(x,1);
                %dimU = size(u,1);
    
            % Compute derivative of acceleration w.r.t. x
%                 daccdx = zeros(1,model.dimX);
%                 f = @(x)get_acceleration_maccepa(x(1:2),[m;u(3)],model);
%                 daccdx(:,1:2)=get_jacobian_fd(f,x(1:2));
%                 f = @(m)get_acceleration_maccepa(x(1:2),[m;u(3)],model);
%                 daccdx(:,[3;6])=get_jacobian_fd(f,m);
%     
%                 % Compute derivative of acceleration w.r.t. u
%                 daccdu = zeros(1,model.dimU);
%                 f = @(u3)get_acceleration_maccepa(x(1:2),[m;u3],model);
%                 daccdu(3)=get_jacobian_fd(f,u(3));
% 
%                 xdot_x = [0, 1, zeros(1,6);...
%                             daccdx          ;...
%                             zeros(6,2), xdot_x2];
%           
%                 xdot_u = [zeros(1,3);...
%                         daccdu; 
%                         xdot_u2,zeros(6,1)];
%             end
        end
        
        function [xdot, xdot_x, xdot_u] = dynamics_with_jacobian(model,x,u)
            xdot = model.dynamics(x,u);
            if nargout > 1
                xdot_x = model.dynx(x,u);
                xdot_u = model.dynu(x,u);
            end
        end
        
        function acc = cmptAcc(model,x,u)
           acc = model.torque_total(x,u)./model.inertia;
        end
        
        function k = stiffness(model, x, u)
           k=0; 
        end
        

        
        %variable damping
        function d = damping(model,duty_circle)
            % duty_circle: 0-1
            d = model.damping_range(1) + duty_circle.*(model.damping_range(2)-model.damping_range(1));
        end
        
        function torque = torque_spring(model, x)
            %sprdis = model.spring_displacement(x,u);
            %spring_force = sprdis*model.spring_constant;
            C = model.pin_displacement;
            B = model.lever_length;
            A = sqrt( C^2 + B^2 - 2*B*C.*cos( x(3,:)-x(1,:) ) );
            phi = x(3,:)-x(1,:);
            torque = model.spring_constant*B*C*sin(phi)*...
                (1+ (model.drum_radius*x(6)-model.A0)/A );
        end
        
        function torque_damp = torque_damping(model,x,u)
            % note size(x,2) == size(u,2)
            torque_damp = x(2,:).*model.damping(u(3,:));
        end
        
        function torque = torque_actuator(model, x, u)
            torque = model.torque_spring(x) - model.torque_damping(x,u);
        end
        
        function torque_total = torque_total(model,x,u)
            torque_total = model.torque_spring(x) - model.torque_damping(x,u)-...
                            model.viscous_friction.*x(2,:);
        end
        
        function sprlength = spring_displacement(model, x)
            C = model.pin_displacement;
            B = model.lever_length;
            A = sqrt(C^2 + B^2 - 2*B*C*cos(x(3)-x(1)));
            sprlength = A+model.drum_radius*x(6)-model.A0; 
        end
        
        function power = net_mechpower(model,x,u)
            % net output mechanical power on motor level
            % note that motor's power has to be non-negative because energy
            % is not recoverable on motor level
            % Note: don't support vector computation currently
            kappa = model.spring_constant;
            B = model.lever_length;
            C = model.pin_displacement;
            r = model.drum_radius;
            A = sqrt(B^2+C^2 - 2*B*C*cos(x(3)-x(1)));
            L = A + model.drum_radius*x(6) - model.A0; 
            tau_motor1 = kappa*B*C*sin(x(3)-x(1))*(1+ (r*x(6)-model.A0)/A );
            tau_motor2 = kappa*L*r;
            power_motor1 = tau_motor1*x(4);
            power_motor2 = tau_motor2*x(7);
             if (power_motor1 <= 0)
                 power_motor1 = 0;
             end
             if (power_motor2 <= 0)
                 power_motor2 = 0;
             end
            d = model.damping(u(3));
            %power_motor1 = sigmf(power_motor1, [10000 0]);
            %power_motor2 = sigmf(power_motor2, [10000 0]);
            
            power = power_motor1 + power_motor2 - d*x(2)^2*model.rege_ratio;
        end
        
        function power = mechpower(model,x,u)
            % net output mechanical power on motor level
            % note that motor's power has to be non-negative because energy
            % is not recoverable on motor level
            % Note: don't support vector computation currently
            kappa = model.spring_constant;
            B = model.lever_length;
            C = model.pin_displacement;
            r = model.drum_radius;
            A = sqrt(B^2+C^2 - 2*B*C*cos(x(3)-x(1)));
            L = A + model.drum_radius*x(6) - model.A0; 
            tau_motor1 = kappa*B*C*sin(x(3)-x(1))*(1+ (r*x(6)-model.A0)/A );
            tau_motor2 = kappa*L*r;
            power_motor1 = tau_motor1*x(4);
            power_motor2 = tau_motor2*x(7);
            if (power_motor1 <= 0)
                power_motor1 = 0;
            end
            if (power_motor2 <= 0)
                power_motor2 = 0;
            end
            %power_motor1 = sigmf(power_motor1, [10000 0]);
            %power_motor2 = sigmf(power_motor2, [10000 0]);
            power = power_motor1 + power_motor2;
        end
    end     
        methods (Static)
            
            function [xdot, xdot_x, xdot_u] = motor_dynamics_3rd(x,u, p)
                z     = zeros(3,1);
                Z     = zeros(3,3);

                Af1 = [0,     1,     0;
                    0,     0,     1;
                    -p(1), -p(2), -p(3)];

                Bf1 = [0;
                    0;
                    p(1)];

                Af2 = [0,     1,     0;
                    0,     0,     1;
                    -p(4), -p(5), -p(6)];
  
                Bf2 = [0;
                   0;
                   p(4)];
 
                A = [Af1,Z;
                Z ,Af2];
 
                B = [Bf1, z;
                z ,Bf2];
    
                xdot    = A * x + B * u;  
                if nargout > 1
                xdot_x  = A;  
                xdot_u  = B; 
                end
            end 
        end
 
    
end

