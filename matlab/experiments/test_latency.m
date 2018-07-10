
%%
if ~exist('pubcmd','var')
    init_ros
    
end

%%
global rate mccssr_raw pubcmd cmdmsg

reset(rate);

cmdmsg.U1 = 2000;
cmdmsg.U2 = 900;
cmdmsg.D1 = 0;
cmdmsg.D2 = 0;
t01 = seconds(rostime('now'));
send(pubcmd, cmdmsg)
t02 =  seconds(rostime('now')) - t01;
Y.u1 = [];
Y.t1 = [];
Y.t2 = [];
now = rostime('now');
now = double(now.Sec)+double(now.Nsec)*10^-9;
while seconds(rostime('now')) - t01 < 0.08
    newdata = receive(mccssr_raw);
    Y.u1 = [Y.u1; newdata.U1];
    Y.t1 = [Y.t1; seconds(newdata.Header.Stamp) - t01];
    Y.t2 = [Y.t2; seconds(rostime('now'))-t01];

    
end