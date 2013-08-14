%
% Default configuration for 2D case
%
% by Olzhas Adiyatov
% 6/10/2013

conf = struct;
conf.delta_goal_point = 7;          % Radius of goal point
conf.delta_near = 15;              % Radius for neighboring nodes
conf.max_step = 7;                % Maximum position change when we add a new node to the tree
conf.half_width = 11;               % kuka width 440mm
conf.half_length = 9;               % kuka length 340mm
conf.init_theta = 0;
conf.turn_theta = 10;
