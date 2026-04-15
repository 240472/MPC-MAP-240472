function [public_vars] = init_kalman_filter(read_only_vars, public_vars)
%INIT_KALMAN_FILTER Summary of this function goes here

public_vars.mu = [mean(read_only_vars.gnss_history)'; pi/2];
public_vars.sigma = eye(3,3).*[(std(read_only_vars.gnss_history).^2)'; 2/3*pi];
%public_vars.sigma = eye(3,3).*[(std(read_only_vars.gnss_history).^2)'; 0];

public_vars.kf.C = [1 0 0; 0 1 0];
public_vars.kf.R = [0.0005 0 0; 0 0.0005 0; 0 0 0.00005];
public_vars.kf.Q = [sqrt(public_vars.sigma(1,1)) 0; 0 sqrt(public_vars.sigma(2,2))];

end

