function [ ut ] = scale_controlSeq( u,t,tsim )
%SCALESEQ_INTIME scale sequence in timescale. Used to scale a sequenec in
%large timescale to smaller timescale
% timestamp: large timescale
% t: smaller timescale
    %Nu = size(u,2);
    Nt = length(tsim);
    ut = zeros(size(u,1),Nt);
    for k = 1:Nt
        ind = findFirst(t,tsim(k));
        if(isnan(ind))
         ut(:,k) = u(:,end);
        else
         ut(:,k) = u(:,ind-1);
        end 
    end

end

