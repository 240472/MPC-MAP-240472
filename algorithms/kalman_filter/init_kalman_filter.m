function [public_vars] = init_kalman_filter(read_only_vars, public_vars)
%INIT_KALMAN_FILTER Summary of this function goes here


map_middle_x = (read_only_vars.map.limits(3)-read_only_vars.map.limits(1))/2;
map_middle_y = (read_only_vars.map.limits(4)-read_only_vars.map.limits(2))/2;

variance = (map_middle_x^2 + map_middle_y^2) / 3;

%public_vars.mu = [mean(read_only_vars.gnss_history)'; pi/2];
public_vars.mu = [map_middle_x; map_middle_y; pi/2];

%public_vars.sigma = eye(3,3).*[(std(read_only_vars.gnss_history).^2)'; 2/3*pi];
%public_vars.sigma = eye(3,3).*[(std(read_only_vars.gnss_history).^2)'; 0];
public_vars.sigma = eye(3,3).*[variance; variance; 2/3*pi];

public_vars.kf.C = [1 0 0; 0 1 0];
public_vars.kf.R = [0.0008 0 0; 0 0.0008 0; 0 0 0.00005];
public_vars.kf.Q = [0.5^2 0; 0 0.5^2];

end

