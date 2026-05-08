function [measurement] = compute_lidar_measurement(map, pose, lidar_config)
%COMPUTE_MEASUREMENTS Summary of this function goes here

 measurement = zeros(1, length(lidar_config));

for i = 1:size(lidar_config,2)
    
    % Intersections between walls and ONE lidar beam
    intersections = ray_cast(pose(1:2), map.walls, lidar_config(i) + pose(3));

    intersections = intersections(~any(isnan(intersections), 2), :);

    
    % Distance to the closest hit wall
    pose_matrix = [ones(size(intersections,1),1)*pose(1), ones(size(intersections,1),1)*pose(2)];
    min_distance = min(vecnorm(intersections - pose_matrix, 2, 2));
    

    % Hitting a wall VS not hitting a wall
    if size(min_distance,1) ~= 0
        measurement(1,i) = min_distance;
    else
        measurement(1,i) = +inf;
    end


end



end

