function [target] = get_target(estimated_pose, path)
%GET_TARGET Summary of this function goes here

persistent waypoint_counter;

if(isempty(waypoint_counter))
    waypoint_counter = 1;
end

if norm(estimated_pose(1:2) - path(waypoint_counter,:)) < 0.5
    waypoint_counter = waypoint_counter + 1;
end

target = path(waypoint_counter,:);

end

