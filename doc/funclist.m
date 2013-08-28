%% RRT*FN Toolbox
% *RRT*FN Toolbox* for solving path/motion planning problems.
% Toolbox provides *RRT*, *RRT** and *RRT*FN* for solving various problems
% (e.g. 2D mobile robot, planar redundant manipulator)
%
% Sampling-based motion planning algorithms got a lot of research attention
% in past decade or so. The main reason of the popularity is an ability
% to solve relatively easy motion planning problems of 2D mobile robot
% as well as hard high dimensional problems both.
%
% *Rapidly-Exporing Random Tree (RRT)* is sampling-based algorithm, solves
% the problem of motion and path planning providing feasible solutions
% however it does not take into account optimality of the solution.
%
% *RRT** is probabilistically optimal variant of RRT which converges to the
% optimal solution asymtotically. Due to the fact that RRT* takes cost of
% the path into consideration, the computational diffuculty increases with
% each node added to tree, a good example of this problem is finding
% neighbors.
%
% *RRT*FN (Fixed Nodes)* is a memory efficient variant of RRT*. It
% inherents all the optimization features of RRT*, but in contrast using
% limited number of nodes i.e. memory is limited.
%
%% 
% <html><body><table style="border: 2px solid orange;"><tr>
% <td style="font-size:12pt;">Please do not change anything in rrt.m,
% rrt_star.m and rrt_star_fn.m files unless it is a bug. <br/>Everything
% you want to add (e.g. new function or modifications of a model)
% please do it creating/editing classes for models.
% e.g. <strong>FNSimple2D.m</strong> or
% <strong>FNRedundantManipulator.m</strong>
% </td></tr></table></body></html>
% 