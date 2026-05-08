function [estimated_pose, estimation_warning_counter] = estimate_pose(public_vars, read_only_vars, particle_weights, estimation_warning_counter)
%ESTIMATE_POSE Summary of this function goes here

if public_vars.kf_enabled

    new_estimated_pose = public_vars.mu';

elseif public_vars.pf_enabled
    
    particles = public_vars.particles;

    N = floor(size(particles,1)*0.1);
    [sorted_w, idx] = sort(particle_weights, 'descend');
    top_idx = idx(1:N);
    top_w   = sorted_w(1:N) / sum(sorted_w(1:N));
    
    x_est     = sum(top_w .* particles(top_idx, 1));
    y_est     = sum(top_w .* particles(top_idx, 2));
    theta_est = atan2(sum(top_w .* sin(particles(top_idx, 3))), ...
                      sum(top_w .* cos(particles(top_idx, 3))));

    new_estimated_pose = [x_est, y_est, theta_est];
    
    if isfinite(public_vars.estimated_pose)
        if vecnorm(public_vars.estimated_pose(1:2) - new_estimated_pose(1:2),2,2) > 0.5
            estimation_warning_counter = read_only_vars.counter + 100;
        end
    end
else

    new_estimated_pose = public_vars.estimated_pose;

end
    
    estimated_pose = new_estimated_pose;

end

