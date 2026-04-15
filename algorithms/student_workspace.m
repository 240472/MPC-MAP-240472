function [public_vars] = student_workspace(read_only_vars,public_vars)
%STUDENT_WORKSPACE Summary of this function goes here

step_size = 0.1;
when_init_counter_N = 300;

line1_y = 2:step_size:7.5;
line2_x = 2:step_size:16;
line3_y = flip(line1_y);

line1_x = ones(1,size(line1_y,2))*2;
line2_y = ones(1,size(line2_x,2))*7.5;
line3_x = ones(1,size(line1_y,2))*16;


% line_seg_x = 5:step_size:10;
% arc_seg_x = 1:step_size:5;
% sine_seg_x = 10:step_size:19;
% saw_seg_x = 15:step_size:19;
% 
% arc_theta = (arc_seg_x-arc_seg_x(1))/(arc_seg_x(end)-arc_seg_x(1))*pi;
% 
% line_seg_y = ones(1,size(line_seg_x,2))*5;
% arc_seg_y = 5 + sin(arc_theta)*(arc_seg_x(end)-arc_seg_x(1))/2;
% arc_seg_x = flip((arc_seg_x(end) + arc_seg_x(1))/2 + cos(arc_theta)*(arc_seg_x(end)-arc_seg_x(1))/2);
% 
% sine_seg_y = sin(pi*(sine_seg_x-sine_seg_x(1)))+5;
% saw_seg_y = ones(1,size(saw_seg_x,2))*5;


%public_vars.path = [line_seg_x, arc_seg_x, sine_seg_x, saw_seg_x; line_seg_y, arc_seg_y, sine_seg_y, saw_seg_y]';




% 8. Perform initialization procedure
if (read_only_vars.counter == when_init_counter_N)
          
    public_vars = init_particle_filter(read_only_vars, public_vars);
    public_vars = init_kalman_filter(read_only_vars, public_vars);

elseif(read_only_vars.counter > when_init_counter_N)

% 9. Update particle filter
public_vars.particles = update_particle_filter(read_only_vars, public_vars);

% 10. Update Kalman filter
[public_vars.mu, public_vars.sigma] = update_kalman_filter(read_only_vars, public_vars);

% 11. Estimate current robot position
public_vars.estimated_pose = estimate_pose(public_vars); % (x,y,theta)
%public_vars.estimated_pose = read_only_vars.mocap_pose; % (x,y,theta)

% 12. Path planning
%public_vars.path = plan_path(read_only_vars, public_vars);
%public_vars.path = [arc_seg_x, line_seg_x, sine_seg_x; arc_seg_y, line_seg_y, sine_seg_y]';
public_vars.path = [line1_x, line2_x, line3_x; line1_y, line2_y, line3_y]';

% 13. Plan next motion command
public_vars = plan_motion(read_only_vars, public_vars);

end

end

