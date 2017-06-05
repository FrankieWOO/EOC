% Dynamics function for the MACCEPA motors.
%
% Calculation of the motor dynamics uses a third-order filter model, based on:
% 
%    Wada, T., Ishikawa, M., Kitayoshi, R., Maruta, I., & Sugie, T. (2009). 
%    Practical modeling and system identification of R/C servo motors. 
%    IEEE International Conference on Control Applications. 
%
function [xdot,xdot_x,xdot_u] = fn_dynamics_servomotor_3rd_order_filter (x,u,p)

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
 xdot_x  = A;  
 xdot_u  = B; 
 
end