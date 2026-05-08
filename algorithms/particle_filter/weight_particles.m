function [weights] = weight_particles(particle_measurements, lidar_distances)
%WEIGHT_PARTICLES Summary of this function goes here

N = size(particle_measurements, 1);

lidar_distance_matrix = ones(N,1)*lidar_distances;

% Weights calculation

difference = lidar_distance_matrix - particle_measurements;
difference(isnan(difference)) = median(difference(isfinite(difference)));

inversed_weights = vecnorm(difference, 2, 2);
inversed_weights(abs(inversed_weights) < 1e-5) = 1e-5;

weights = ones(N,1)./inversed_weights;

% Normalization
weights = weights/sum(weights);

end
