% State space dynamics function for the Kawato arm model.
%
% in: 
%     x     = state [position; velocity]
%     u     = command [muscle activations]
%     model = model struct
%
% out:
%     xdot = state change
%
function xdot = f_kawato_arm ( x, u, model )

qddot = qddot_kawato_arm ( x(1:2), x(3:4), u, model );
xdot  = [x(3:4); qddot];

