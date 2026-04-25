function [path] = plan_path(read_only_vars, public_vars)
%PLAN_PATH Summary of this function goes here

persistent init_path 

if(isempty(init_path))
    init_path = 1;
end

planning_required = 0;

% something like this will be used for the final application
% if (sum(diag(public_vars.sigma(1:2,1:2))) < 0.1 && init_path)
%     planning_required = 1;
%     init_path = 0;
% end

% always turns planning just once, at the beginning
if init_path
    planning_required = 1;
    init_path = 0;
end

if planning_required
    
    path = astar(read_only_vars, public_vars);
    
    path = smooth_path(path);
    
else
    
    path = public_vars.path;
    
end

end

