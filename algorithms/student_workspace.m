function [public_vars] = student_workspace(read_only_vars,public_vars)
%STUDENT_WORKSPACE Summary of this function goes here

persistent kidnapped_recovery_counter 


when_init_counter_N = 300;      % 1 -> Robot starts immediatelly, N > 1 -> Robot collects data before starting
particle_count = 200;
new_path = 0;

if ~public_vars.pf_enabled
    public_vars.particles = [];
end

if isnan(read_only_vars.gnss_position)
    public_vars.kf_enabled = 0;
else
    public_vars.kf_enabled = 1;
end

% 8. Perform initialization procedure

if (read_only_vars.counter == 1)
    kidnapped_recovery_counter = 0;
end

if (~public_vars.kf_enabled && ~public_vars.pf_enabled)

    public_vars = init_particle_filter(read_only_vars, public_vars, particle_count);

elseif (read_only_vars.counter == when_init_counter_N)
    
    public_vars = init_kalman_filter(read_only_vars, public_vars);

end


if (read_only_vars.counter >= 1)

    % 9. Update particle filter
    [public_vars, particle_weights] = update_particle_filter(read_only_vars, public_vars);

    if ~public_vars.kf_enabled
        public_vars.motion_vector = [0.02, -0.02];
    end

end


if (read_only_vars.counter >= when_init_counter_N)

    % 10. Update Kalman filter
    [public_vars.mu, public_vars.sigma] = update_kalman_filter(read_only_vars, public_vars);
    
    % 11. Estimate current robot position
    [public_vars.estimated_pose, kidnapped_recovery_counter] = estimate_pose(public_vars, read_only_vars, particle_weights, kidnapped_recovery_counter); % (x,y,theta)
    
    % 12. Path planning
    if read_only_vars.counter == when_init_counter_N || (kidnapped_recovery_counter == read_only_vars.counter)
        public_vars.path = plan_path(read_only_vars, public_vars);
        new_path = 1;
    end
    % 13. Plan next motion command
    if isnan(public_vars.estimated_pose)
        public_vars.motion_vector = [0, 0];

    elseif kidnapped_recovery_counter <= read_only_vars.counter
        public_vars = plan_motion(read_only_vars, public_vars, new_path);

    else
        public_vars.motion_vector = [0.02, -0.02];

    end

end

