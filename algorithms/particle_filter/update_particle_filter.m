function [public_vars, weights] = update_particle_filter(read_only_vars, public_vars)
%UPDATE_PARTICLE_FILTER Summary of this function goes here

particles = public_vars.particles;
weights = [];
handover = 0;

if ~public_vars.kf_enabled && ~public_vars.pf_enabled
    public_vars.pf_enabled = 1;
    if (read_only_vars.counter ~= 1)
        handover = 1;
    end
end

if public_vars.kf_enabled
    public_vars.pf_enabled = 0;
end

if public_vars.pf_enabled

    % I. Prediction
    for i=1:size(particles, 1)
        particles(i,:) = predict_pose(particles(i,:), public_vars.motion_vector, read_only_vars);
    end
    
    % II. Correction
    measurements = zeros(size(particles,1), length(read_only_vars.lidar_config));
    for i=1:size(particles, 1)
        measurements(i,:) = compute_lidar_measurement(read_only_vars.map, particles(i,:), read_only_vars.lidar_config);
    end
    weights = weight_particles(measurements, read_only_vars.lidar_distances);
    
    % III. Resampling
    particles = resample_particles(particles, weights);
    
    % IV. Adding fresh particles
    if handover
        persistent_particles_perc = 0.25;
    else
        persistent_particles_perc = 0.8;
    end
       
    random_particles_count = (size(particles,1)) - floor(size(particles,1) * persistent_particles_perc) + 1;
    
    temp_public_struct = init_particle_filter(read_only_vars, public_vars, random_particles_count, handover);
    
    particles(floor(size(particles,1) * persistent_particles_perc):end,:) = temp_public_struct.particles;
end

public_vars.particles = particles;

end

