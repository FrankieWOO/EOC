function [ Y ] = exetrajs( U, varargin )
%EXETRAJS execute multiple trajectories
%   input:  U - cell array of commands
%   output: Y - cell array of recorsd
    %global rate
    Y = cell(size(U));
    global pubcmd cmdmsg
    
for i=1:size(U,1)
    u = U{i,1};
    %uraw = convert_cmd_to_raw(u);
    %y = zeros(2, size(u,2)+1);
    
    y = exetraj(u);
    
    Y{i, 1} = y;
    
    disp('Execution of traj done. Press keys to reset...')
    pause
    % reset to init position
    cmdmsg.U1 = 1500;
    cmdmsg.U2 = 900;
    cmdmsg.D1 = 0;
    cmdmsg.D2 = 0;
    send(pubcmd, cmdmsg);
    disp('Reseted to initial position. Press keys to continue...')
    pause
    
end


end

