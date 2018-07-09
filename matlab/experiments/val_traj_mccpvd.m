function [ traj ] = val_traj_mccpvd( model, task, traj )
%
t = 0:task.dt:task.T; %control timestamp
sdt = 0.001;
tsim = 0:sdt:task.T; % simulation timestamp
usim = scale_controlSeq(traj.u, t, tsim);
psim.solver = 'rk4';
psim.dt = sdt;
xsim = simulate_feedforward(traj.x(:,1), task.f, usim, psim);

power_out = model.power_out(xsim(:,1:end-1),usim);
power_in = model.power_in(xsim(:,1:end-1), usim);
power_in1= model.power_in1(xsim(:,1:end-1), usim);
power_in2= model.power_in2(xsim(:,1:end-1), usim);
power_rege = model.power_rege(xsim(:,1:end-1),usim);
power_damp = model.power_damp(xsim(:,1:end-1),usim);
Es = model.energy_spring(xsim,usim);

traj.target = task.target;
traj.tsim = tsim;
traj.usim = usim;
traj.xsim = xsim;
traj.power_out = power_out;
traj.power_in = power_in;
traj.power_in1= power_in1;
traj.power_in2= power_in2;
traj.power_rege = power_rege;
traj.power_damp = power_damp;
traj.Es = Es;
traj.Eout = sum(power_out)*sdt;
traj.Ein = sum(power_in)*sdt;
traj.Ein1 = sum(power_in1)*sdt;
traj.Ein2 = sum(power_in2)*sdt;
traj.Edamp = sum(power_damp)*sdt;
traj.Erege = sum(power_rege)*sdt;

end

