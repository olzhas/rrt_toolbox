%% rrt_star_fn
% *RRT*FN* is a memory efficient variant of RRT*. It inherents all
% the optimization features of RRT*, but in contrast using limited
% number of nodes i.e. memory is limited.
%
%% Syntax
%  problem = rrt_star(map, max_iter, max_nodes, is_benchmark, rand_seed, variant)
%  function returns the object of the respective class with the result
%
%% Input
%  map           -- struct with appropriate fields (developer of 
%                the class provides more information on this topic)
%  max_iter      -- number of iteration to solve the problem
%  max_nodes     -- the maximum allowed number of nodes added to the tree
%  is_benchmark  -- if true saves snapshots of the tree in a special directory
%                boolean variable
%  rand_seed     -- a random seed 
%  variant       -- what class to choose, class used defines the problem space
%% Output
% *problem* is an object of an appropriate problem space representing class 
%