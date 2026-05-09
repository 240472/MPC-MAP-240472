function [estimated_pose, kidnapped_recovery_counter] = estimate_pose(public_vars, read_only_vars, particle_weights, kidnapped_recovery_counter)
%ESTIMATE_POSE Summary of this function goes here

if public_vars.kf_enabled

    new_estimated_pose = public_vars.mu';

elseif public_vars.pf_enabled
    
    particles = public_vars.particles;

    N = floor(size(particles,1)*0.5);
    [sorted_w, idx] = sort(particle_weights, 'descend');
    particles_sorted = particles(idx, :);
    top_idx = idx(1:N);
    top_w   = sorted_w(1:N) / sum(sorted_w(1:N));
    
    % Spočítej median theta z top částic
    x_median = median(particles_sorted(top_idx,1));
    y_median = median(particles_sorted(top_idx,2));
    theta_median = atan2(median(sin(particles_sorted(top_idx, 3))), ...
                         median(cos(particles_sorted(top_idx, 3))));


    % Odfiltruj částice jejichž theta je daleko od medianu
    max_pos_diff = 0.5;
    dist = sqrt((particles(top_idx,1) - x_median).^2 + ...
            (particles(top_idx,2) - y_median).^2);

    max_theta_diff = 0.3; % radiany, ~17 stupňů
    dtheta = abs(atan2(sin(particles_sorted(top_idx, 3) - theta_median), ...
                       cos(particles_sorted(top_idx, 3) - theta_median)));
    
    valid_xy = dist < max_pos_diff;
    valid_theta = dtheta < max_theta_diff;
    valid = valid_xy & valid_theta;


    % Použij jen validní částice
    top_idx   = top_idx(valid);
    top_w     = top_w(valid) / sum(top_w(valid)); % renormalizuj

    x_est     = sum(top_w .* particles_sorted(top_idx, 1));
    y_est     = sum(top_w .* particles_sorted(top_idx, 2));
    theta_est = atan2(sum(top_w .* sin(particles_sorted(top_idx, 3))), ...
                      sum(top_w .* cos(particles_sorted(top_idx, 3))));
    
    new_estimated_pose = [x_est, y_est, theta_est];

    if isfinite(public_vars.estimated_pose)
        if vecnorm(public_vars.estimated_pose(1:2) - new_estimated_pose(1:2),2,2) > 0.5
            kidnapped_recovery_counter = read_only_vars.counter + 100;
        end
    end
else

    new_estimated_pose = public_vars.estimated_pose;

end
if ~isnan(new_estimated_pose)    
    if size(read_only_vars.est_position_history,1) < 2 || public_vars.kf_enabled
        estimated_pose = new_estimated_pose;
    else
        alpha = 0.3; % 0 = hodně hladké, 1 = žádné vyhlazení

        estimated_pose(1:2) = alpha * new_estimated_pose(1:2) + (1-alpha) * read_only_vars.est_position_history(end,1:2);
        
        dtheta = atan2(sin(new_estimated_pose(3) - read_only_vars.est_position_history(end,3)), ...
               cos(new_estimated_pose(3) - read_only_vars.est_position_history(end,3)));
        
        estimated_pose(3) = read_only_vars.est_position_history(end,3) + alpha * dtheta; 
    end
else
    estimated_pose = public_vars.estimated_pose;
end

