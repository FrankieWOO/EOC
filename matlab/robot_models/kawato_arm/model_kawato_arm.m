% Planar 2 degree of freedom human arm model based on Kawato's human arm model.
%
%  out:
%      model.L           - Link length
%           .I           - Inertia
%           .M           - Mass
%           .L_g1        - Centre of gravity
%           .umax,umin   - Max/min. commands
%           .dt          - sample time for control
%           .robot       - Matlab Robotics Toolbox model
%           .q0          - Joint angle at rest position (u=0)
%           .A           - Moment arms
%           .gk          - Muscle elasticity coefficients
%           .k0          - Initial muscle elasticity
%           .gb          - Muscle viscosity coefficients
%           .b0          - Initial muscle viscosity 
%           .gr          - difference with rest length when maximally contracted (assuming u_max=ones)
%           .lm_l0       - Constant for standard posture
%
function model = model_kawato_arm

model.name = 'Kawato Arm';

model.dimQ = 2; % number of joints
model.dimU = 6; % number of actuators (commands) 

% Model geometry and dynamics parameters.
% Choices for these are based on the paramters in the Katayama/Kawato paper.
model.L  =  0.3*ones(2,1);  % link lengths
model.I  = [0.0477;0.0588]; % inertia
model.M  = [1.59; 1.44];    % mass
model.Lg = [0.18; 0.21];    % center of gravity
model.g  = 0; % gravity

model.qmax  =  pi/2*ones(2,1); % joint limits
model.qmin  = -pi/2*ones(2,1); % TODO % what is a good value for this?
model.umax  =  ones(6,1); % maximum motor command
model.umin  = zeros(6,1); % minimum motor command

model.viscous_friction = 0;
model.coulomb_friction = 0;

% Muscle parameters vector; note the indices
% 1 = shoulder flexor
% 2 = shoulder extensor
% 3 = elbow flexor
% 4 = elbow extensor
% 5 = two-joint flexor
% 6 = two-joint extensor
model.A   = 0.01*[4.0 -4.0 0 0 2.8 -2.8;...  % constant moment arm matrix
                  0 0 2.5 -2.5 3.5 -3.5]';   % negative sign added
                                             % to extensor muscles
model.gk  = ones(6,1)*1621.6;                % elasticity coefficients
model.k0  = ones(6,1)*810.8;                 % initial elasticity
model.gb  = ones(6,1)*108.1;                 % viscosity coefficients
model.b0  = ones(6,1)*54.1;                              % initial viscosity 
model.gr  = 0.01*[3.491; 3.491; 2.182; 2.182; 5.498; 5.498]; % muscle activation constant

% now define lm-l0 according to the standard posture
A = model.A;
K0 = diag(model.k0);
P = A'*K0;
Pinv = P'*inv(P*P'); % pseudo inverse of P=A'*K0

q0 = [45;70]*pi/180;
delta0 = [0;0;0;0;0;0]; % defines pre-tension

% constant for standard posture
model.lm_l0 = Pinv*A'*K0*A*q0 + (eye(6)-Pinv*P)*delta0;

