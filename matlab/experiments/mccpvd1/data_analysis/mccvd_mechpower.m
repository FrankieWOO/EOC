function [ res ] = mccvd_mechpower( x,u,p)
%   MW_POWER_MCC calculate the mechanical work power of MACCEPA-VD
%   assume the Mechanical Work = change of kinetic E + change of spring
%   potential + output energy flow
%   p:
%   model - maccepavd model
%   recovery_ratio - (0,1)
%   x - state vector containing motors' states
%   power_model - to choose power estimation methods
    
    if isfield(p, 'power_model')
        power_model = p.power_model;
    else
        power_model = 8;
    end
    
    model = p.model;
    if isfield(p, 'alpha')
        %ratio = 1 - p.recovery_ratio;
        alpha = p.alpha;
    else
        %ratio = 1;
        alpha = 0;
    end
    kappa = model.spring_constant; 
    r = model.drum_radius;
    C = model.pin_displacement; B = model.lever_length; L0 = abs(C-B);
    A = sqrt(B^2+C^2 - 2*B*C*cos(x(3)-x(1)));
    %L: spring length
    L = r*x(6) + A - L0;
            
    switch power_model
        
            
        case 8
            % net output mechanical power on motor level
            % note that motor's power has to be non-negative because energy
            % is not recoverable on motor level
            tau_motor1 = kappa*B*C*sin(x(3)-x(1))*(1+ (r*x(6)-L0)/A );
            tau_motor2 = kappa*L*r;
            power_motor1 = tau_motor1*x(4);
            power_motor2 = tau_motor2*x(7);
            if (power_motor1 <= 0)
                power_motor1 = 0;
            end
            if (power_motor2 <= 0)
                power_motor2 = 0;
            end
            damp = model.getDamp(x,u);
            res = power_motor1 + power_motor2 - alpha*damp*(x(2)^2);
            
        case 9
            % output mechanical power = output spring power + spring energy
            % change 
            P1 = kappa*L* B*C*sin( x(3)-x(1) ) * (x(4)-x(2)) /A ;
            P2 = kappa*L*r*x(7) ;
            damp = model.getDamp(x,u);
            res = x(2).* model.getTorqueAct(x,u ) + P1+P2 - ...
                alpha*damp*x(2)^2;
            
        case 10
            % 
            P1 = kappa*L* B*C*sin( x(3)-x(1) ) * (x(4)-x(2)) /A ;
            P2 = kappa*L*r*x(7) ;
            damp = model.getDamp(x,u);
            power = x(2).* model.getTorqueAct(x,u ) + P1+P2 ;
            if (power <= 0)
                power = 0;
            end
            res = power - alpha*damp*x(2)^2;
            
        case 11
            % square of spring force
            res = (kappa*L)^2;
            
    end
end


