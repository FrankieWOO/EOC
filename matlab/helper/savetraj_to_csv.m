function [  ] = savetraj_to_csv( csvfile, traj )
%
%   
    if isstruct(traj)
        if isfield(traj, 'u')
            u1 = traj.u(1,:)';
            u2 = traj.u(2,:)';
            u3 = traj.u(3,:)';
            t  = traj.t(1:length(u1))';
        end
    end
    data = table(t,u1,u2,u3);
    writetable(data, csvfile);
end

