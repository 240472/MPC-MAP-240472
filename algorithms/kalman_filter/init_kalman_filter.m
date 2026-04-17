function [public_vars] = init_kalman_filter(read_only_vars, public_vars)
%INIT_KALMAN_FILTER Summary of this function goes here

if read_only_vars.counter == 1      % For case without waiting period

    map_middle_x = (read_only_vars.map.limits(3)-read_only_vars.map.limits(1))/2;
    map_middle_y = (read_only_vars.map.limits(4)-read_only_vars.map.limits(2))/2;
    public_vars.mu = [map_middle_x; map_middle_y; pi/2];
    
    variance = (map_middle_x^2 + map_middle_y^2) / 3;
    public_vars.sigma = eye(3,3).*[variance; variance; (pi^2)/3];

    public_vars.kf.Q = [0.5001^2 0; 0 0.4880^2];

else        % For case with waiting period
    
    %Task 4
    public_vars.mu = [mean(read_only_vars.gnss_history)'; pi/2];        
    public_vars.sigma = eye(3,3).*[(std(read_only_vars.gnss_history).^2)'; (pi^2)/3];

    public_vars.kf.Q = [public_vars.sigma(1,1)^2 0; 0 public_vars.sigma(2,2)^2];
    
    %Task 3
    % public_vars.mu = [2; 2; pi/2];     
    % public_vars.sigma = zeros(3,3);

end


public_vars.kf.C = [1 0 0; 0 1 0];
public_vars.kf.R = [8e-4 0 0; 0 8e-4 0; 0 0 5e-5];       % Better for long distances with high speed
public_vars.kf.R = [8e-7 0 0; 0 8e-7 0; 0 0 5e-5];       % Better for careful and slow movement

end

