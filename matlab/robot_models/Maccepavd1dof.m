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
        
        
        %%%% physical design
        % physical joint constriants
        % note that the physical constriant of joint is the rechable point
        % not the controllable point
        qmax = 135*pi/180; %
        qmin = -135*pi/180; %
        % gear ratio from theta1 to q
        gear1 = 4/3; %44/33; 
        
        spring_constant = 231;
        link_length = 0.15;% link length
        link_mass = 0.09;
        
        
        lever_length = 0.036; %B
        pin_displacement = 0.135; %C
        A0
        drum_radius = 0.015;
        
        servo1_mass = 0.09;
        
       
        %variable damping range
        % 110255 : 4.2400e-04
        damping_range = [0, 0.1070];
        rege_ratio = 0;
        
        
        %%%% ---- physical design
        
        %%%% servo motor specification, reflected at gearbox output shaft
        inertia_m1 = 0.0099;
        K_m1 = 0.3811; % torque constant
        D_m1 = 0.2009;
        R_m1 = 0.8222;
        inertia_m2 = 0.0099;
        K_m2 = 0.3811;
        D_m2 = 0.2009;
        R_m2 = 0.8222;
        %%%%
        
        %%%% dynamical properties
        inertia = 0.00137% calculated inertia
        % frictions
        viscous_friction = 0.013 %0.0025;
        coulomb_friction = 0;
        gravity_constant = 0;
        
        %%%%
        %%%% control input limits
        umax = [pi/3; 2*pi/3; 1] ;
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
            
            servo_param.p = [14000;1150;38;14000;1150;38];
            %servo_param.p = [ 2650;455;28;1152;244;20];
            %model.fnMotorDyn =@(x,u)model.motor_dynamics_3rd(x,u,servo_param.p); 
            
            %servo_param = load('maccepa_servomotor_model.mat');
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
        
        function k = stiffness(model, x)
            C = model.pin_displacement;
            B = model.lever_length;
            A = sqrt( C^2 + B^2 - 2*B*C.*cos( x(3,:)/model.gear1-x(1,:) ) );
            phi = x(3,:)/model.gear1-x(1,:);
            kappa = model.spring_constant;
            r = model.drum_radius;
            k=  kappa*B*C*cos(phi)*( 1 + (r*x(6) - model.A0)/A ) - ...
                kappa*(B*C*sin(phi))^2*( r*x(6) - model.A0 )/A^1.5 ;
        end
        
        function tau_m = tau_m1(model, x)
            tau_l = model.torque_spring(x)/model.gear1;
            tau_m = tau_l + model.D_m1*x(4) + model.inertia_m1*x(5);
        end
        
        function tau_m = tau_m2(model, x)
            tau_l = model.spring_displacement(x)*model.spring_constant;
            tau_m = tau_l + model.D_m2*x(7) + model.inertia_m2*x(8);
        end
        %variable damping
        function d = damping(model,duty_circle)
            % duty_circle: 0-1
            % linear on duty-circle
            d = model.damping_range(1) + duty_circle.*(model.damping_range(2)-model.damping_range(1));
        end
        
        function torque = torque_spring(model, x)
            %sprdis = model.spring_displacement(x,u);
            %spring_force = sprdis*model.spring_constant;
            C = model.pin_displacement;
            B = model.lever_length;
            A = sqrt( C^2 + B^2 - 2*B*C.*cos( x(3,:)/model.gear1-x(1,:) ) );
            phi = x(3,:)/model.gear1-x(1,:);
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
            A = sqrt(C^2 + B^2 - 2*B*C*cos(x(3)/model.gear1-x(1)));
            sprlength = A+model.drum_radius*x(6)-model.A0; 
        end
        
        function [p_total, p1, p2  ] = total_power(model, x, ~)
            tau_m1 = model.tau_m1(x);
            tau_m2 = model.tau_m2(x);
            I1 = tau_m1/model.K_m1;
            I2 = tau_m2/model.K_m2;
            p1e = I1^2*model.R_m1;
            p2e = I2^2*model.R_m2;
            p1m = tau_m1*x(4);
            p2m = tau_m2*x(7);
            if p1m < 0, p1m =0; end
            if p2m < 0, p2m =0; end
            p_total = p1e + p2e + p1m + p2m ;
            p1 = p1e + p1m;
            p2 = p2e + p2m;
        end
        function [p_ntotal, p1, p2  ] = net_total_power(model, x, u)
            tau_m1 = model.tau_m1(x);
            tau_m2 = model.tau_m2(x);
            I1 = tau_m1/model.K_m1;
            I2 = tau_m2/model.K_m2;
            p1e = I1^2*model.R_m1;
            p2e = I2^2*model.R_m2;
            p1m = tau_m1*x(4);
            p2m = tau_m2*x(7);
            if p1m < 0, p1m =0; end
            if p2m < 0, p2m =0; end
            d = model.damping(u(3));
            p_ntotal = p1e + p2e + p1m + p2m - d*x(2)^2*model.rege_ratio;
            p1 = p1e + p1m;
            p2 = p2e + p2m;
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
            A = sqrt(B^2+C^2 - 2*B*C*cos(x(3)/model.gear1-x(1)));
            L = A + model.drum_radius*x(6) - model.A0; 
            
            tau_s = kappa*B*C*sin(x(3)/model.gear1-x(1))*(1+ (r*x(6)-model.A0)/A );
            tau_l1 = tau_s/model.gear1;
            tau_l2 = kappa*L*r;
            power_motor1 = tau_l1*x(4);
            power_motor2 = tau_l2*x(7);
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
        
        function power = output_mechpower(model,x,~)
            % net output mechanical power on motor level
            % note that motor's power has to be non-negative because energy
            % is not recoverable on motor level
            % Note: don't support vector computation currently
            kappa = model.spring_constant;
            B = model.lever_length;
            C = model.pin_displacement;
            r = model.drum_radius;
            A = sqrt(B^2+C^2 - 2*B*C*cos(x(3)/model.gear1-x(1)));
            L = A + model.drum_radius*x(6) - model.A0; 
            tau_motor1 = kappa*B*C*sin(x(3)/model.gear1-x(1))*(1+ (r*x(6)-model.A0)/A );
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

