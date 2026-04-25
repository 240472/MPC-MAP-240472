function [new_path] = smooth_path(old_path)
%SMOOTH_PATH Summary of this function goes here

smooth_path = old_path;

iter = 100;
alpha = 0.1;
beta = 0.3;

for i = 2:size(old_path,1) - 1
    for k = 1:iter
        similarity = old_path(i,:) - smooth_path(i,:);
        neighbor = smooth_path(i - 1,:) + smooth_path(i + 1,:) - 2*smooth_path(i,:);

        smooth_path(i,:) = smooth_path(i,:) + alpha*similarity + beta*neighbor;   
    end

end

new_path = smooth_path;

end

