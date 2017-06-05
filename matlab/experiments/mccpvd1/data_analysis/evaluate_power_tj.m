function [ powers ] = evaluate_power_tj( x,u,p )
%POWER_TRAJ return the power along a trajectory
% use p.power_model to choose power estimator
%   p:
%   model - maccepavd model
%   recovery_ratio - (0,1)
%   x - state vector containing motors' states
%   power_model - to choose power estimation methods
    N = size(u,2);
    powers = zeros(1,N);
    for i=1:N
        powers(i) = mccvd_mechpower(x(:,i),u(:,i),p);
    end

end


