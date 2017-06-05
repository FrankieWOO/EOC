function [ work ] = cmptWork( x,u,p )
%CMPTWORK Summary of this function goes here
%   Detailed explanation goes here
    powers = evaluate_power_tj(x,u , p);
    
    work = sum(powers)*p.dt;
end

