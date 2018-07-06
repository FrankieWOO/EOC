
%%
simmccpa_example
cmds = {result0.u;result1.u;result2.u;result3.u;result4.u};

%%
if ~exist('pubcmd','var')
    init_ros
    
end

%%
records = cell(size(cmds));
for i=1:size(cmds,1)
    u = cmds{i,1};
    uraw = convert_cmd_to_raw(u);
    y = zeros(2, size(u,2)+1);
    reset(rate);
    for j=1:size(uraw,2)
        ssrdata = receive(mccssr, 1);
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
    records{i, 1} = y;
    
    % reset to init position
    cmdmsg.U1 = 1500;
    cmdmsg.U2 = 900;
    cmdmsg.D1 = 0;
    cmdmsg.D2 = 0;
    send(pubcmd, cmdmsg);
    pause
end

%% plot
final_time = T;
t4plot = 0:0.02:final_time;
figure
subplot(2,1,1)
hold on
for k=1:size(records,1)
    yy = records{k,1};
    plot(t4plot, yy(1,:))
end
hold off
title('Joint position')
%legend('C.D.','dynamic', 'regenerative','hybrid','fixed damping')

subplot(2,1,2)
hold on
for k=1:size(records,1)
    yy = records{k,1};
    plot(t4plot, yy(2,:))
end
hold off
title('Rege. Current')
%legend('C.D.','dynamic', 'regenerative','hybrid','fixed damping')

%%
Erege_record = zeros(5,1);
for k=1:size(records,1)
    yy = records{k,1};
    Erege_record(k,1) = sum((yy(2,:)/1000).^2*24)*0.02;
end


