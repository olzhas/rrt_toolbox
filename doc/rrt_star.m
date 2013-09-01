%% rrt_star
% *RRT** is probabilistically optimal variant of RRT which converges
% to the optimal solution asymtotically. Due to the fact that RRT* takes
% cost of the path into consideration, the computational diffuculty
% increases with each node added to tree, a good example of this
% problem is finding neighbors.
%
%% Syntax
%  problem = rrt_star(map, max_iter, is_benchmark, rand_seed, variant)
%  function returns the object of the respective class with the result
%
%% Input
%  map           -- struct with appropriate fields (developer of 
%                the class provides more information on this topic)
%  max_iter      -- number of iteration to solve the problem
%  is_benchmark  -- if true saves snapshots of the tree in a special directory
%                boolean variable
%  rand_seed     -- a random seed 
%  variant       -- what class to choose, class used defines the problem space
%% Output
% *problem* is an object of an appropriate problem space representing class 
%