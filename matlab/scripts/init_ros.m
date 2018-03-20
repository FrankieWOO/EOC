%% init ros node, publisher and subscriber
rosinit
global rate pubcmd cmdmsg mccssr
rate = rosrate(50);
rate.OverrunAction = 'slip';

[pubcmd, cmdmsg] = rospublisher('/command_raw','maccepavd/CommandRaw');
mccssr = rossubscriber('/sensors');
