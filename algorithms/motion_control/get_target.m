function [target] = get_target(estimated_pose, path, new_path)
%GET_TARGET Summary of this function goes here

persistent waypoint_counter;

if isempty(waypoint_counter) || new_path
    waypoint_counter = 1;
end

if waypoint_counter > size(path,1)
    waypoint_counter = size(path,1);
end

if norm(estimated_pose(1:2) - path(waypoint_counter,:)) < 0.25
    waypoint_counter = waypoint_counter + 1;

    

end

if waypoint_counter <= size(path,1)
    target = path(waypoint_counter,:);
else
    target = path(end,:);

end

