%% How to create new model for solving path/motion planning problem
% 
% <html><body><table style="border: 2px solid orange;"><tr>
% <td style="font-size:12pt;">Please do not change anything in rrt.m,
% rrt_star.m and rrt_star_fn.m
% </td></tr></table></body></html>
% 
%%
% At the designing stage of the toolbox Object Oriented Approach employed to
% increase the generality of certain peaces of code. 
%%
% <<schema.png>>
%
%%
% This schematic figure shows how files relate.
%
%% Creating new model
%
% rrt.m, rrt_star.m and rrt_star_fn.m contain all necessary code to
% properly run algorithms. In order to add new model, a developer should
% add new class with appropriate methods.

%% Methods to be implemented for RRT
% 
% * *sample()*
% * *nearest( _new_node_ )*
% * *steer( _nearest_node_ind_ , _new_node_ )*
% * *obstacle_collision( _new_node_ , _nearest_node_ )*
% * *insert_node( _nearest_node_ , _new_node_ )*
% * *plot()*

%% Methods to be implemented for RRT*
% 
% * *sample()*
% * *nearest( _new_node_ )*
% * *steer( _nearest_node_ind_ , _new_node_ )*
% * *obstacle_collision( _new_node_ , _nearest_node_ )*
% * *neighbors( _new_node_ , _nearest_node_ind_ )*
% * *chooseParent( _neighbors_ , _nearest_node_ind_ , _new_node_ )*
% * *insert_node( _min_node_ind_ , _new_node_ )*
% * *rewire( _new_node_ind_ , _neighbors_ , _min_node_ind_ )*
% * *plot()*

%% Methods to be implemented for RRT*FN
% 
% * *sample()*
% * *nearest( _new_node_ )*
% * *steer( _nearest_node_ind_ , _new_node_ )*
% * *obstacle_collision( _new_node_ , _nearest_node_ )*
% * *neighbors( _new_node_ , _nearest_node_ind_ )*
% * *chooseParent( _neighbors_ , _nearest_node_ind_ , _new_node_ )*
% * *reuse_node( _min_node_ , _new_node_ )*
% * *insert_node( _min_node_ind_ , _new_node_ )*
% * *rewire( _new_node_ind_ , _neighbors_ , _min_node_ind_ )*
% * *best_path_evaluate()*
% * *forced_removal()*
% * *plot()*

%% For more details please read <matlab:edit('FNSimple2D.m') FNSimple2D.m> or <matlab:edit('FNRedundantManipulator.m') FNRedundantManipulator.m>
%