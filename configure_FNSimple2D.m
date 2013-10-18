%
% Default configuration for 2D case
%
% by Olzhas Adiyatov
% 6/10/2013

conf = struct;
conf.delta_goal_point = 1;          % Radius of goal point
conf.delta_near = 1.5;              % Radius for neighboring nodes
conf.max_step = 0.5;                % Maximum position change when we add a new node to the tree
conf.bin_size = 7;