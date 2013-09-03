%% RRT* for 3 DOF Mobile Robot
% 
% <html><body><table style="border: 2px solid orange;"><tr>
% <td style="font-size:12pt;">Do not change anything in rrt.m,
% rrt_star.m and rrt_star_fn.m
% </td></tr></table></body></html>
% 

%% Getting started
% Create *main_3dof_mobile_rrt_star.m* file. You can create m-file with any other name, that is
% perfectly okay, this will not affect to the solution of a path/motion
% planning problem. All the sources could be found in examples/ directory
% of the distribution.
%

%% Step 1: Choosing the map 
% Firstly, we should define what map to use. It is done by defining map
% structure the sample code is goes after.
%
%   map = struct('name', 'bench_june1.mat', 'start_point', [-14.5 -7.5], 'goal_point', [10 -0.65]);
%

%%
%
% * *name field* defines the file name in maps/ directory
% * *start_point* field defines where the initial point of the problem
% * *goal_point* field define where is the goal point on the map
%

%% Step 2: Setting maximum number of iterations
%
%   max_iter = 20e3;
%
% * *max_iter* variable defines how much iteration should be done to solve
% the path planning problem.

%% Step 3: Do we have to benchmark 
%
%   is_benchmark = false;
%
% * *is_benchmark* enables benchmarking. For more details please read the
% sources of rrt.m, rrt_star.m and rrt_star_fn.m

%% Step 4: Setting random seed
%
%   rand_seed = 40;
%
% * *rand_seed* variable is a random seed for random number generator, it
% is used for sampling nodes. It is usually used for benchmarking. We set
% the same map, however the random seed is different for the same map. You
% can use *now* if you don't care about random seed.
%
%   rand_seed = now;
%

%% Step 5: Choosing the class (model) we want 
% 
%   variant = 'FNSimple3D';
%
% * *variant* defines from what class we should instantiate the object. In
% other words it defines what model we choose for application of RRT.
%
% *FNSimple3D* is a name of a class which contains all necessary methods
% and fields in order to represent simple 3 DOF Mobile Robot model.

%% Step 6: RRT*
%
%   rrt_star(map, max_iter, is_benchmark, rand_seed, variant);
%
% Line above runs RRT with given parameters. In addition, *rrt* function
% returns the class object with a certain solution.

%% Sources of *main_3dof_mobile_rrt_star.m*
% Press 
% <matlab:edit('examples/main_3dof_mobile_rrt_star.m') here>
% to play with example code.
%
%   % 3 DOF mobile robot example.
%   % by Almaskhan Baimyshev 
%   % 08/28/2013
%   
%   map = struct('name', 'bench_june1.mat', 'start_point', [-14.5 -7.5], 'goal_point', [10 -0.65]);
%   max_iter = 20e3;
%   is_benchmark = false;
%   rand_seed = 40;
%   variant = 'FNSimple3D';
%   result = rrt_star(map, max_iter, is_benchmark, rand_seed, variant);
%
