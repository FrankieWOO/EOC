function [ y ] = exetraj( u )
    % execute a single trajectory
    % requires global variables rate, mccssr, pubcmd, cmdmsg
    % input: u - 
    global rate mccssr pubcmd cmdmsg
    uraw = convert_cmd_to_raw(u);
    y = zeros(2, size(u,2)+1);
    reset(rate);
    for j=1:size(uraw,2)
        ssrdata = receive(mccssr,1);
        y(1,j) = ssrdata.JointPosition;
        y(2,j) = ssrdata.RegeCurrent;
    
        cmdmsg.U1 = uraw(1,j);
        cmdmsg.U2 = uraw(2,j);
        cmdmsg.D1 = uraw(3,j);
        cmdmsg.D2 = uraw(4,j);
        send(pubcmd, cmdmsg)
        waitfor(rate);
    end
    ssrdata = receive(mccssr, 1);
    y(1,end) = ssrdata.JointPosition;
    y(2,end) = ssrdata.RegeCurrent;
    
end

