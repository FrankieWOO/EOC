function [ y ] = convert_sequence_time( x, t, newt )
%CONVERT_SEQUENCE_TIME Summary of this function goes here
%   Detailed explanation goes here
    y = zeros(size(newt));
    for i=1:length(newt)
        index = findFirst(t, newt(i));
        h1 = newt(i)-t(index);
        h2 = t(index+1) - newt(i);
        y(i) = x(index)*h1/(h1+h2) + x(index+1)*h2/(h1+h2);
    end

end

