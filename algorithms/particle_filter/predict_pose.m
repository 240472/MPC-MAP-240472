function [new_pose] = predict_pose(old_pose, motion_vector, read_only_vars)
%PREDICT_POSE Summary of this function goes here


Ts = read_only_vars.sampling_period;

mu = 0; 
sigma = 0.02;    
prediction_noise = mu + sigma * randn(3,1);


if(~(abs(motion_vector(1) - motion_vector(2)) < 1e-5))
    d = read_only_vars.agent_drive.interwheel_dist;
    v_r = motion_vector(1);
    v_l = motion_vector(2);
    
    % turning radius and angular velocity
    R     = (d/2) * (v_r + v_l) / (v_r - v_l);
    omega = (v_r - v_l) / d;
    
    % instantaneous center of curvature
    ICC = [old_pose(1) - R * sin(old_pose(3));
           old_pose(2) + R * cos(old_pose(3))];
    
    % rotation matrix around ICC
    rot = [cos(omega*Ts), -sin(omega*Ts), 0;
           sin(omega*Ts),  cos(omega*Ts), 0;
           0,              0,             1];
    
    % pose relative to ICC, then rotate and translate back
    pose_rel  = [old_pose(1) - ICC(1);
                 old_pose(2) - ICC(2);
                 old_pose(3)];
    
    new_pose = rot * pose_rel + [ICC(1); ICC(2); omega*Ts];
else
    v = (motion_vector(1) + motion_vector(2)) / 2;
    new_pose = [old_pose(1) + v * Ts * cos(old_pose(3));
                old_pose(2) + v * Ts * sin(old_pose(3));
                old_pose(3)];
end


new_pose = new_pose + prediction_noise;

if new_pose(1) < read_only_vars.map.limits(1)
    new_pose(1) = read_only_vars.map.limits(1) + 0.01;
elseif new_pose(1) > read_only_vars.map.limits(3)
    new_pose(1) = read_only_vars.map.limits(3) - 0.01;
end

if new_pose(2) < read_only_vars.map.limits(2)
    new_pose(2) = read_only_vars.map.limits(2) + 0.01;
elseif new_pose(2) > read_only_vars.map.limits(4)
    new_pose(2) = read_only_vars.map.limits(4) - 0.01;
end

new_pose(3) = atan2(sin(new_pose(3)), cos(new_pose(3)));

end

