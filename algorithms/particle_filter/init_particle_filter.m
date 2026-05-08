function [public_vars] = init_particle_filter(read_only_vars, public_vars, particle_count, handover)
%INIT_PARTICLE_FILTER Summary of this function goes here

arguments
    read_only_vars
    public_vars
    particle_count = read_only_vars.max_particles;   % default if not provided
    handover = 0;
end

if handover
        
    low_bound_x = public_vars.mu(1) - sqrt(public_vars.sigma(1,1));
    high_bound_x = public_vars.mu(1) + sqrt(public_vars.sigma(1,1));
        
    low_bound_y = public_vars.mu(2) - sqrt(public_vars.sigma(2,2));
    high_bound_y = public_vars.mu(2) + sqrt(public_vars.sigma(2,2));
        
    low_bound_theta = public_vars.mu(3) - sqrt(public_vars.sigma(3,3));
    high_bound_theta = public_vars.mu(3) + sqrt(public_vars.sigma(3,3));

else

    % Bounds for particles, so they won't generate outside of a map
    low_bound_x = read_only_vars.map.limits(1);
    high_bound_x = read_only_vars.map.limits(3);
        
    low_bound_y = read_only_vars.map.limits(2);
    high_bound_y = read_only_vars.map.limits(4);
        
    low_bound_theta = 0;
    high_bound_theta = 2*pi;
    
end

init_particles_x = low_bound_x + (high_bound_x - low_bound_x) * rand(1, particle_count);
init_particles_y = low_bound_y + (high_bound_y - low_bound_y) * rand(1, particle_count);
init_particles_theta = low_bound_theta + (high_bound_theta - low_bound_theta) * rand(1, particle_count);

public_vars.particles = [init_particles_x', init_particles_y', init_particles_theta'];

end

