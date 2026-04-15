function [new_mu, new_sigma] = ekf_predict(mu, sigma, u, kf, sampling_period)
%EKF_PREDICT Summary of this function goes here

vt = u(1);
omega = u(2);

G = [1 0 -sin(mu(3))*vt*sampling_period; 
     0 1 cos(mu(3))*vt*sampling_period; 
     0 0 1];

new_mu = [mu(1)+cos(mu(3))*vt*sampling_period;
          mu(2)+sin(mu(3))*vt*sampling_period;
          mu(3)+omega*sampling_period];

new_sigma = G*sigma*G'+ kf.R;

end

