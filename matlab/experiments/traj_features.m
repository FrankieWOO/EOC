function [ res ] = traj_features( model, task, x, u, t)
%TRAJ_FEATURES calculate all the features that can be used to evaluate and
%characterise a trajectory and its performance
    
    stiffness = zeros(size(t));
    damping = zeros(size(t)); % variable damping
    b = zeros(size(t));
    damping_ratio = zeros(size(t));
    p_outmech = zeros(size(t)); % tau_load * v
    p_mech = zeros(size(t)); % tau_m * v
    p1_mech = zeros(size(t));
    p2_mech = zeros(size(t));
    p_elec = zeros(size(t));
    p1_elec = zeros(size(t));
    p2_elec = zeros(size(t));
    p_rege = zeros(size(t));
    p_netelec = zeros(size(t));
    res.tau_spring = zeros(size(t));
    %cost = evaluate_trajectory_cost_fh();
    for i = 1:length(t)-1
         stiffness(i) = model.stiffness(x(:,i));
         damping(i) = model.damping(x,u(:,i));
         b(i) = damping(i) + model.Df;
         damping_ratio(i) = model.damping_ratio(x(:,i),u(:,i));
         p_outmech(i) = model.output_mechpower(x(:,i),u(:,i));
         [p_mech(i),p1_mech(i),p2_mech(i)] = model.power_mech(x(:,i),u(:,i));
         [p_elec(i),p1_elec(i),p2_elec(i)] = model.power_elec(x(:,i),u(:,i));
         p_rege(i) = model.power_charge(x(:,i),u(:,i));
         p_netelec(i) = max(0,p1_elec(i))+max(0,p2_elec(i))-p_rege(i);
         res.tau_spring(i) = model.torque_spring(x(:,i));
    end
    
    E_elec = sum( max(0, p1_elec) )*task.dt + task.dt*sum( max(0, p2_elec) );
    E_mech = sum( max(0, p1_mech) )*task.dt + sum( max(0, p2_mech) )*task.dt;
    E_netelec = E_elec - sum(p_rege)*task.dt;
    E_netmech = E_mech - sum(p_rege)*task.dt;
    
    res.cost_accuracy = sum((task.target-x).^2 )*task.dt;
    res.stiffness = stiffness;
    res.damping = damping;
    res.b = b;
    res.damping_ratio = damping_ratio;
    res.p_outmech = p_outmech;
    res.p_mech = p_mech;
    res.p1_mech = p1_mech;
    res.p2_mech = p2_mech;
    res.p_elec = p_elec;
    res.p1_elec = p1_elec;
    res.p2_elec = p2_elec;
    res.p_rege = p_rege;
    res.p_netelec = p_netelec;
    res.E_elec = E_elec;
    res.E_mech = E_mech;
    res.E_netelec= E_netelec;
    res.E_netmech= E_netmech;
    res.E_rege = sum(p_rege)*task.dt;
    res.E_outmech = sum( max(0,p_outmech))*task.dt;
end

