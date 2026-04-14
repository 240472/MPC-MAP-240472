function [new_mu, new_sigma] = kf_measure(mu, sigma, z, kf)
%KF_MEASURE Summary of this function goes here

Kt = sigma*kf.C'*((kf.C*sigma*kf.C'+kf.Q)^(-1));

new_mu = mu + Kt*(z-kf.C*mu);
new_sigma = (eye(size(Kt*kf.C))-Kt*kf.C)*sigma;

end

