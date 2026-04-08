function [measurement] = compute_lidar_measurement(map, pose, lidar_config)
%COMPUTE_MEASUREMENTS Summary of this function goes here

 measurement = zeros(1, length(lidar_config));

for i = 1:size(lidar_config,2)
    angle = (i-1)*pi/4;
    intersections = ray_cast(pose(1:2), map.walls, angle + pose(3));

    pose_matrix = [ones(size(intersections,1),1)*pose(1), ones(size(intersections,1),1)*pose(2)];
    measurement(1,i) = min(vecnorm(intersections - pose_matrix, 2, 2));
end



end

