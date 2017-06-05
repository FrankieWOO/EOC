function [ ut ] = scaleSeq_intime( u,timestamp,t )
%SCALESEQ_INTIME scale sequence in timescale. Used to scale a sequenec in
%large timescale to smaller timescale
    %Nu = size(u,2);
    Nt = length(t);
    ut = zeros(size(u,1),Nt);
    for k = 1:Nt
        ind = findFirst(timestamp,t(k));
        if(isnan(ind))
         ut(:,k) = u(:,end);
        else
         ut(:,k) = u(:,ind-1);
        end 
    end

end

