function [ dx ] = servo_2ndmodel( x,u,p )
%SERVO_2NDMODEL(X,U,P) Summary of this function goes here
%   Detailed explanation goes here
    A = [ 0 1;
          -p^2 -2*p];
    B = [0; p^2];
    dx = A*x + B*u;

end

