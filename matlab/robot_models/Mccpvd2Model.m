classdef Mccpvd2Model
    %   MACCEPA-VD
    %   Detailed explanation goes here
    %   1DoF MACCEPAVD
    %   x: q ; dq ; theta1 ; theta2; dtheta1; dtheta2;
    %   
    %   
    
    properties
        robot_type = 'Mccpvd2Model';
        
        cdt = 0.02 % controller time step
        
        dimQ = 2% dimension of joints
        dimX = 12% dimension of states
        dimU = 6% dimension of controls
        
        
        %%%% physical design
        % physical joint constriants
        % note that the physical constriant of joint is the rechable point
        % not the controllable point
        qmax = [135*pi/180, 135*pi/180]; %
        qmin = [-135*pi/180,-135*pi/180]; %
        
        L = [0.3, 0.33]; % link length
        mass = [1.4, 1];
        com = [0.11, 0.16];
        moi = [0.025, 0.045];
        
        B = [0.05, 0.025;
            0.025, 0.05];
        
        %%%%
        a1 = moi(1) + moi(2) + mass(2)*L(1)^2;
        a2 = mass(2)*L(1)*com(2);
        a3 = moi(2);
        
        %%%%
        
        
       
        
        
        %%%% ---- physical design
        
        %%%% servo motor specification, reflected at gearbox output shaft
%         inertia_m1 = 0.0099;
%         K_m1 = 0.3811; % torque constant
%         D_m1 = 0.2009 ;
%         R_m1 = 0.8222;
%         inertia_m2 = 0.0099;
%         K_m2 = 0.3811;
%         D_m2 = 0.2009;
%         R_m2 = 0.8222;
%         
%         alpha_servo = 25; % Fan: fit from data
        %%%%
        
        %%%% dynamical properties
        % Fan: estimated by data  %calculated inertia: 0.00135
        % 
        
        
        
        
        %%%%
        %%%% control input limits
        % u1 and u2 are defined in rad; round to int and below the physical
        % limit to protect the servo
        umax = [ 1; 2; 1] ;
        umin = [-1; 0; 0] ;
        %%%%
        
        
        
        %%%% function handlers
        fnMotorDyn
        dynx
        dynu
        %%%%
        
        %%%% symbolic
%         symDyn
%         symDynx
%         symDynu
%         
        
        %%%%
        actuator1
        actuator2
        %%%%
        
        
    end
    
    properties (Access = private)
        dynxfull
        dynufull
    end
    
    methods
        function model = Mccpvd2Model()
            
            % gear ratio from theta1 to q
%             gear1 = 3/4; %Nin = 44, Nout = 33; 
%         
%             spring_constant = 231;
%             lever_length = 0.036; %B
%             pin_displacement = 0.135; %C
%             
%             drum_radius = 0.015;
            %variable damping range
            % 110255 : 0.00848
%             damping_range = [0, 0.00848];
            
            actparam.ratio_load = 1;
            actparam.gear_d = 100;
            model.actuator1 = ActMccpvd(actparam);
            model.actuator2 = ActMccpvd(actparam);
            
            
            
            %model.inertia = model.inertia_l + model.actuator.Id;
            
            %model = model.init_symfuns();

        end
        
        function [M] = M_matrix(model, q)
            m11 = model.a1 + 2*model.a2*cos(q(2));
            m12 = model.a3 + model.a2*cos(q(2));
            m21 = model.a3 + model.a2*cos(q(2));
            m22 = model.a3;
            M = [m11, m12;
                m21, m22];
        end
        
        function [C] = C_matrix(model, q, qdot)
            c1 = -qdot(2)*(2*qdot(1)+qdot(2));
            c2 = qdot(1);
            C = [c1;c2]*model.a2*sin(q(2));
        end
        
        
        % state-space forward dynamics
        function [xdot]= dynamics(model, x, u)
            M = model.M_matrix(x(1:2));
            C = model.C_matrix(x(1:2), x(3:4));
            
            
            qddot = model.cmptAcc(x,u);
            qdot = x(2,:);
            xdot1 = [qdot; qddot] ;

            % motor dynamics
            xdot2 = model.motor_dynamics_2nd(x(3:end),u(1:2));
            
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
        
        function [xdot, xdot_x, xdot_u]= dynamics_with_jacobian_fd(model, x, u)
            qddot = model.cmptAcc(x,u);
            qdot = x(2,:);
            xdot1 = [qdot; qddot] ;

            % motor dynamics
            [xdot2, xdot2_x, xdot2_u] = model.motor_dynamics_2nd(x(3:end),u(1:2));
            
            xdot  = [xdot1;
                    xdot2];
            % current motor positions
            %m     = [x(3);x(4)] ; 
            if nargout > 1
            % Compute xdot_x, xdot_u using finite differences
                %delta = 1e-6;
                %dimX = size(x,1);
                %dimU = size(u,1);
    
            % Compute derivative of acceleration w.r.t. x
                 %daccdx = zeros(1,model.dimX);
                 f = @(x)model.cmptAcc(x,u);
                 daccdx = get_jacobian_fd(f,x);
                 if iscolumn(daccdx), daccdx = daccdx';end
                 % Compute derivative of acceleration w.r.t. u
                 %daccdu = zeros(1,model.dimU);
                 f = @(u)model.cmptAcc(x,u);
                 daccdu = get_jacobian_fd(f,u);
                 if iscolumn(daccdu), daccdu = daccdu'; end
                 xdot_x = [0, 1, zeros(1,4);
                             daccdx          ;
                             zeros(4,2), xdot2_x];
           
                 xdot_u = [zeros(1,3);
                         daccdu; 
                         xdot2_u,zeros(4,1)];
             end
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
            k =  model.actuator.stiffness(x(1),x(3),x(4));
        end
        
        
        %variable damping
        function d = damping(model,~,u)
            % duty_circle: 0-1
            % linear on duty-circle
            d = model.actuator.damping(u(3));
        end
        
        function torque = torque_spring(model, x,~)
            %sprdis = model.spring_displacement(x,u);
            %spring_force = sprdis*model.spring_constant;
            
            torque = model.actuator.torque_spring(x(1),x(3),x(4));
        end
        
        function torque_damp = torque_damping(model,x,u)
            % note size(x,2) == size(u,2)
            torque_damp = model.actuator.torque_damping(x(2),u(3));
        end
        
        function torque = torque(model, x, u)
            % joint torque
            % q, qdot, m1, m2, dc
            torque1 = model.actuator.torque(x(1),x(3),x(5),x(7),u(3));
            torque2 = model.actuator.torque(x(1),x(2),x(3),x(4),u(3));
            torque = [torque1; torque2];
        end
        
        function torque_total = torque_total(model,x,u)
            torque_total = model.actuator.torque(x(1),x(2),x(3),x(4),u(3)) - model.Df*x(2);
        end
        
        function tau_m = tau_m1(model, x, u)
            % tau_m of servo1 at servo's shaft
            tau_l = model.torque_spring(x)/model.actuator.gear;
            p = model.alpha_servo;
            accel = p^2*(u(1)-x(3)) - 2*p*x(5);
            
            tau_m = tau_l + model.D_m1*x(5) + model.inertia_m1*accel;
            %tau_m = tau_l;
        end
        
        function tau_m = tau_m2(model, x, u)
            % tau_m of servo2 at servo's shaft
            tau_l = model.actuator.spring_displacement(x(1),x(3),x(4))*model.actuator.Ks;
            p = model.alpha_servo;
            accel = p^2*(u(2)-x(4)) - 2*p*x(6);
            
            tau_m = tau_l + model.D_m2*x(6) + model.inertia_m2*accel;
        end
        function [p_total, p1, p2  ] = total_power(model, x, u)
            tau_m1 = model.tau_m1(x,u);
            tau_m2 = model.tau_m2(x,u);
            I1 = tau_m1/model.actuator.K1;
            I2 = tau_m2/model.actuator.K2;
            p1e = I1^2*model.actuator.R1;
            p2e = I2^2*model.actuator.R2;
            p1m = tau_m1*x(5);
            p2m = tau_m2*x(6);
            if p1m < 0, p1m =0; end
            if p2m < 0, p2m =0; end
            
            p_total = p1e + p2e + p1m + p2m ;
            p1 = p1e + p1m;
            p2 = p2e + p2m;
        end
        function [p_ntotal, p1, p2  ] = net_total_power(model, x, u)
            d = model.damping(u(3));
            [p, p1, p2] = model.total_power(x,u);
            p_ntotal = p - d*x(2)^2*model.rege_ratio;
        end
        function [power, p1, p2] = power_mech(model,x,u)
            tau_m1 = model.tau_m1(x,u);
            tau_m2 = model.tau_m2(x,u);
            p1 = tau_m1*x(5);
            p2 = tau_m2*x(6);
            power = p1 + p2;
        end
        function [power, p1_elec, p2_elec, p1_diss, p2_diss] = power_elec(model, x, u)
            tau_m1 = model.tau_m1(x,u);
            tau_m2 = model.tau_m2(x,u);
            I1 = tau_m1/model.K_m1;
            I2 = tau_m2/model.K_m2;
            p1_diss = I1^2*model.R_m1;
            p2_diss = I2^2*model.R_m2;
            p1_mech = tau_m1*x(5);
            p2_mech = tau_m2*x(6);
            p1_elec = p1_diss + p1_mech;
            p2_elec = p2_diss + p2_mech;
            power = p1_elec + p2_elec;
        end
        function p = power_charge(model, x, u)
            p = model.actuator.p_damp_charge(x(2),u(3));
        end
        function power = net_mechpower(model,x,u)
            % net output mechanical power on motor level
            % note that motor's power has to be non-negative because energy
            % is not recoverable on motor level
            % Note: don't support vector computation currently
            power_outmech = model.output_mechpower(x,u);
            d = model.actuator.damping(u(3));
            %power_motor1 = sigmf(power_motor1, [10000 0]);
            %power_motor2 = sigmf(power_motor2, [10000 0]);
            
            power = power_outmech - d*x(2)^2*model.rege_ratio;
        end
        
        function [power, power_motor1, power_motor2] = output_mechpower(model,x,u)
            % output mechanical power on motor level
            % note that motor's power has to be non-negative because energy
            % is not recoverable on motor level
            % Note: don't support vector computation currently
            kappa = model.actuator.Ks;
            B = model.actuator.B;
            C = model.actuator.C;
            r = model.actuator.r;
            gear = model.actuator.gear;
            A0 = model.actuator.A0;
            A = sqrt(B^2+C^2 - 2*B*C*cos(x(3)/gear-x(1)));
            L = A + r*x(4) - A0; 
            
            tau_s = model.torque_spring(x,u);
            tau_l1 = tau_s/gear;
            tau_l2 = kappa*L*r;
            power_motor1 = tau_l1*x(5);
            power_motor2 = tau_l2*x(6);
            %if (power_motor1 <= 0)
            %    power_motor1 = 0;
            %end
            %if (power_motor2 <= 0)
            %    power_motor2 = 0;
            %end
            %power_motor1 = sigmf(power_motor1, [10000 0]);
            %power_motor2 = sigmf(power_motor2, [10000 0]);
            power = power_motor1 + power_motor2;
        end
        
        function [ xdot, xdot_x, xdot_u ] = motor_dynamics_2nd(model, x, u)
            p = model.alpha_servo;
            A = [ 0, 0, 1, 0;
                0, 0, 0, 1;
                -p^2, 0, -2*p, 0;
                0, -p^2, 0, -2*p];
            B = [ 0, 0;
                0, 0;
                p^2, 0;
                0, p^2];
            xdot = A*x + B*u;
            
            if nargout > 1
                xdot_x  = A;  
                xdot_u  = B; 
            end
            
        end
        
        function dr = damping_ratio(model, x, u)
            b = model.Df + model.damping(x,u);
            k = model.stiffness(x);
            dr = b/2*sqrt(k*model.inertia);
        end
        
    end     
        methods (Static)
            
%             function [xdot, xdot_x, xdot_u] = motor_dynamics_3rd(x,u, p)
%                 z     = zeros(3,1);
%                 Z     = zeros(3,3);
% 
%                 Af1 = [0,     1,     0;
%                     0,     0,     1;
%                     -p(1), -p(2), -p(3)];
% 
%                 Bf1 = [0;
%                     0;
%                     p(1)];
% 
%                 Af2 = [0,     1,     0;
%                     0,     0,     1;
%                     -p(4), -p(5), -p(6)];
%   
%                 Bf2 = [0;
%                    0;
%                    p(4)];
%  
%                 A = [Af1,Z;
%                 Z ,Af2];
%  
%                 B = [Bf1, z;
%                 z ,Bf2];
%     
%                 xdot    = A * x + B * u;  
%                 if nargout > 1
%                 xdot_x  = A;  
%                 xdot_u  = B; 
%                 end
%             end
%             
            
            
            
        end
        
    
end


