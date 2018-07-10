function [ Y ] = exetraj_debug( u )
    % for debug and test time latency
    % execute a single trajectory
    % requires global variables rate, mccssr, pubcmd, cmdmsg
    % input: u - 
    global rate mccssr_raw pubcmd cmdmsg
    uraw = convert_cmd_to_raw(u);
    y = zeros(2, size(u,2)+1);
    reset(rate);
    for j=1:size(uraw,2)
        cmdmsg.U1 = uraw(1,j);
        cmdmsg.U2 = uraw(2,j);
        cmdmsg.D1 = uraw(3,j);
        cmdmsg.D2 = uraw(4,j);
        send(pubcmd, cmdmsg)
        
        ssrdata = receive(mccssr_raw, 1);
        y(1,j) = ssrdata.JointPosition;
        y(2,j) = ssrdata.RegeCurrent;
        y(3,j) = ssrdata.Header.Stamp.Sec;
        y(4,j) = ssrdata.U1;
        %y(1,j) = ssrdata.JointSensor;
        %y(2,j) = ssrdata.RegeCurrent;
        waitfor(rate);
    end
    ssrdata = receive(mccssr, 1);
    y(1,end) = ssrdata.JointPosition;
    y(2,end) = ssrdata.RegeCurrent;
    Y.joint_position = y(1,:);
    Y.rege_current = y(2,:);
end

