% 2D mobile robot example.
% by Olzhas Adiyatov 
% 08/28/2013

map = struct('name', 'bench_june1.mat', 'start_point', [-12.5 -5.5], 'goal_point', [7 -3.65]);
max_iter =  9.570e3;
is_benchmark = false;
rand_seed = 40;
variant = 'FNSimple2D';
result = rrt_star(map, max_iter, is_benchmark, rand_seed, variant);