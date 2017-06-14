function [ dx ] = servo_3rdmodel( x,u,p )
%SERVO_3RDMODEL Summary of this function goes here
%   Detailed explanation goes here
    A = [ 0 1 0;
          0 0 1;
         -p(1) -p(2) -p(3)];
    B = [0; 0; p(1)];
    dx = A*x + B*u;

end

