function [new_particles] = resample_particles(particles, weights)
%RESAMPLE_PARTICLES Summary of this function goes here


[weights_sorted, idx] = sort(weights, 'ascend');
particles_sorted = particles(idx, :);
new_particles = zeros(size(particles));

N = size(particles,1);

index = randi(size(particles,1), 1);

for i = 1:N
    beta = 2*max(weights_sorted)*rand(1,1);
    while weights_sorted(index) < beta
        beta = beta - weights_sorted(index);
        index =  mod(index, N) + 1;
    end
    new_particles(i,:) = particles_sorted(index,:);
end

end

