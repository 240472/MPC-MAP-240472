function [public_vars] = plan_motion(read_only_vars, public_vars, new_path)
%PLAN_MOTION Summary of this function goes here

% I. Pick navigation target

if isempty(public_vars.path)
    return
end

target = get_target(public_vars.estimated_pose, public_vars.path, new_path);


% II. Compute motion vector

d = read_only_vars.agent_drive.interwheel_dist;

%Feedback linearization method
xr = public_vars.estimated_pose(1);
yr = public_vars.estimated_pose(2);
theta = public_vars.estimated_pose(3);

epsilon = 0.1;
xp = xr+epsilon*cos(theta);
yp = yr+epsilon*sin(theta);

kappa = norm(target-[xp,yp]);
xp_dot = kappa*(target(1)-xp);
yp_dot = kappa*(target(2)-yp);

v = xp_dot*cos(theta)+yp_dot*sin(theta);
omega = (-xp_dot*sin(theta)+yp_dot*cos(theta))/epsilon;

omega_max = 10.0;  
omega = max(-omega_max, min(omega_max, omega));

v = max(0.1, v * (1 - 0.5 * abs(omega) / omega_max));
v = min(v, read_only_vars.agent_drive.max_vel);

omega_r = (2*v+omega*d)/2;
omega_l = (2*v-omega*d)/2;


public_vars.motion_vector = [omega_r, omega_l]; %prava,leva
%public_vars.motion_vector = [0.2, 0.2]; %prava,leva


end