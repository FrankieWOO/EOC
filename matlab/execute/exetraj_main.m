
rate = rosrate(50);
rate.OverrunAction = 'slip';

[pubcmd, cmdmsg] = rospublisher('/command_raw','maccepavd/CommandRaw');

%% test
reset(rate);
for i=1:10
    cmdmsg.U1 = 1500;
    cmdmsg.U2 = 1200;
    cmdmsg.D1 = 0;
    cmdmsg.D2 = 0;
    send(pubcmd, cmdmsg)
    waitfor(rate);
end

%%
tj_hybrid = load('g20/tj_hybrid.mat');
u = tj_hybrid.result3.u;
uraw = convert_cmd_to_raw(u);

%%
reset(rate);
for i=1:size(uraw,2)
    cmdmsg.U1 = uraw(1,i);
    cmdmsg.U2 = uraw(2,i);
    cmdmsg.D1 = uraw(3,i);
    cmdmsg.D2 = uraw(4,i);
    send(pubcmd, cmdmsg)
    waitfor(rate);
end
