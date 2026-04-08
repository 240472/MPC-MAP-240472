function [public_vars] = init_particle_filter(read_only_vars, public_vars)
%INIT_PARTICLE_FILTER Summary of this function goes here

particle_count = read_only_vars.max_particles;

low_bound_x = read_only_vars.map.limits(1);
high_bound_x = read_only_vars.map.limits(3);
init_particles_x = low_bound_x + (high_bound_x - low_bound_x) * rand(1, particle_count);

low_bound_y = read_only_vars.map.limits(2);
high_bound_y = read_only_vars.map.limits(4);
init_particles_y = low_bound_y + (high_bound_y - low_bound_y) * rand(1, particle_count);

low_bound_theta = 0;
high_bound_theta = 2*pi;
init_particles_theta = low_bound_theta + (high_bound_theta - low_bound_theta) * rand(1, particle_count);

public_vars.particles = [init_particles_x', init_particles_y', init_particles_theta'];

end

