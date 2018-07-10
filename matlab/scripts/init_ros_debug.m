%% init ros node, publisher and subscriber
rosinit
global rate pubcmd cmdmsg mccssr mccssr_raw
rate = rosrate(50);
rate.OverrunAction = 'slip';
%%
[pubcmd, cmdmsg] = rospublisher('/command_raw','maccepavd/CommandRaw');
mccssr = rossubscriber('/sensors');
mccssr_raw = rossubscriber('/sensors_raw');
