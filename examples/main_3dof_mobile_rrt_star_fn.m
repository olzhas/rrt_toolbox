% 3 DOF mobile robot example.
% by Almaskhan Baimyshev
% 08/28/2013

map = struct('name', 'bench_june1.mat', 'start_point', [-14.5 -7.5], 'goal_point', [10 -0.65]);
max_iter = 20e3;
max_nodes = 3e3;
is_benchmark = false;
rand_seed = 40;
variant = 'FNSimple3D';
result = rrt_star_fn(map, max_iter, max_nodes, is_benchmark, rand_seed, variant);