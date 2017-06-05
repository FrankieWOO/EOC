function [ ind ] = findFirst( a, B )
%FINDFIRST find the first element in a that > B
%   search from the first element in a and stop when a_i > B
    n = length(a);
    ind = NaN;
    for i=1:n
       if(a(i)>B)
           ind = i;
           return
       end
    end

end

