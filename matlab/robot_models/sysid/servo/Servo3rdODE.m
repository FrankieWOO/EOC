function [ A,B,C,D ] = Servo3rdODE( p1, p2, p3, Ts )
%SERVO3RDODE ODE representation of 3rd order servomotor dynamics
    A = [ 0 1 0;
          0 0 1;
         -p1 -p2 -p3];
    B = [0; 0; p1];
    C = eye(3);
    D = [0;0;0];
    if Ts>0 % sample model with smaple time Ts
        s = expm([ [A B]*Ts; zeros(1,4)] );
        A = s(1:3,1:3);
        B = s(1:3,4);
    end
end

