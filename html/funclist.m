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
