function [ ut ] = scale_controlSeq( u,t,tsim )
%   scale sequence into another timescale. Used to scale a sequenec in
%   large timescale to smaller timescale, for simulation
%   tsim: scaled time
%   t: original time
%   @F. WU

    Nt = length(tsim);
    ut = zeros(size(u,1),Nt-1);
    for k = 1:Nt-1
        ind = findFirst(t,tsim(k));
        if(isnan(ind))
         ut(:,k) = u(:,end);
        else
         ut(:,k) = u(:,ind-1);
        end 
    end

end

