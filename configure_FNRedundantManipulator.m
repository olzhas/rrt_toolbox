%
% Default configuration for Redundant Robotic Manipulator nDOF
%
% by Olzhas Adiyatov
% 6/10/2013

conf = struct;

conf.delta_goal_point = 3;                                          % Radius of goal point
conf.num_link = 6;                                                  % Number of links in the manipulator model
conf.max_ang = [pi; 3*pi/4*ones(conf.num_link-1,1)];                % Maximum joint angle for each joint
conf.min_ang = [0 ;-3*pi/4*ones(conf.num_link-1,1)];                % Minimum joint angle for each joint
conf.delta_ang_max = 10 * pi / 180;                                 % Maximum joint angle change at each iteration
conf.delta_ang_neighbor = 30 * pi / 180;                            % Maximum joint angle change at each neighbor search
conf.disp_interval = 100;                                           % Interval for progress monitoring
conf.init_conf = [pi/2; zeros(conf.num_link-1,1)];                  % Initial configuration of the manipulator model
conf.step_div = 4;                                                  % Number of transition states between nodes, used for collision detection

conf.len_link    = 14*ones(conf.num_link, 1);                       % Array storing the length of the links
conf.height_link = 12 *ones(conf.num_link, 1);                      % Array storing the height of the links
conf.width_link  = 4 *ones(conf.num_link, 1);                       % Array storing the width of the links
