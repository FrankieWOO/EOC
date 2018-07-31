
num_targets = 25;
init_position = 0;
joint_limit = [-pi/3, pi/3];
%u2 = pi/6; % u2 fixed at pi/6
target_list = zeros(num_targets, 1);

% generate random targets
q0 = init_position;
for iter = 1:num_targets
    rd = random('unif', joint_limit(1), joint_limit(2));
    while abs(rd - q0)<pi/6
        rd = random('unif', joint_limit(1), joint_limit(2));
    end
    target_list(iter) = rd;
    
    q0 = rd; % assume we can get to the target
end