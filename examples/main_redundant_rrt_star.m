% RRT for Redundant manipulator example.
% by Olzhas Adiyatov 
% 08/28/2013

map = struct('name', 'bench_redundant_3.mat', 'start_point', [0 0], 'goal_point', [35 35]);
max_iter = 20e3;
is_benchmark = false;
rand_seed = 40;
variant = 'FNRedundantManipulator';
result = rrt(map, max_iter, is_benchmark, rand_seed, variant);
