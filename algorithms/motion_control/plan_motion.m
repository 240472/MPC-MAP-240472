function [public_vars] = plan_motion(read_only_vars, public_vars, new_path)
%PLAN_MOTION Summary of this function goes here

% I. Pick navigation target

if isempty(public_vars.path)
    return
end

target = get_target(public_vars.estimated_pose, public_vars.path, new_path);


% II. Compute motion vector

d = read_only_vars.agent_drive.interwheel_dist;

% Feedback linearization method
xr = public_vars.estimated_pose(1);
yr = public_vars.estimated_pose(2);
theta = public_vars.estimated_pose(3);

epsilon = 0.1;
xp = xr + epsilon * cos(theta);
yp = yr + epsilon * sin(theta);

dist   = norm(target - [xp, yp]);
kappa  = dist;  % gain = vzdálenost

xp_dot = kappa * (target(1) - xp);
yp_dot = kappa * (target(2) - yp);

%v_raw = xp_dot * cos(theta) + yp_dot * sin(theta);
omega = (-xp_dot * sin(theta) + yp_dot * cos(theta)) / epsilon;


% Limits
v_max     = read_only_vars.agent_drive.max_vel;
omega_max = 10.0;

omega = max(-omega_max, min(omega_max, omega));

if public_vars.pf_enabled
    brake_radius = 1.2;
else
    brake_radius = 2;
end

v_dist = v_max * min(1, dist / brake_radius);
v_turn = v_max * (1 - 0.8 * abs(omega) / omega_max);

v = min(v_dist, v_turn);
v = max(0, v);

if dist > brake_radius * 0.5
    v = max(0.10, v);
end

omega_r = (2 * v + omega * d) / 2;
omega_l = (2 * v - omega * d) / 2;


public_vars.motion_vector = [omega_r, omega_l]; %prava,leva


end