
target = pi/3;
parm = [];
parm.has_gravity = false;
pact = [];
pact.max_stiffness = 10;
pact.max_damping = 10;

robot = Pendulum1(parm,pact);

ptask = [];
ptask.target = target;
ptask.w_t = 1;
ptask.w_e = 1;
ptask.w_tf = 1;
ptask.w_r = 1;

task = pendulum1_reach(ptask);
%%
f = robot.dynamics