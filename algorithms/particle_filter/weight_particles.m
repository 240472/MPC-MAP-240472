function [weights] = weight_particles(particle_measurements, lidar_distances)
%WEIGHT_PARTICLES Summary of this function goes here

N = size(particle_measurements, 1);
lidar_distance_matrix = zeros(N,8);

for i = 1:8
    lidar_distance_matrix(:,i) = ones(N,1)*lidar_distances(i);
end

weights = ones(N,1)./vecnorm(lidar_distance_matrix - particle_measurements, 2, 2);
weights = weights/sum(weights);      %kinda dont know how to normalize 

end

