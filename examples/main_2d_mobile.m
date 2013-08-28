% 2D mobile robot example.
% by Olzhas Adiyatov 
% 08/28/2013

map = 'first.mat';
max_iter = 10e4;
is_benchmark = true;
rand_seed = 40;
variant = 'FNSimple2D';
result = rrt(map, max_iter, is_benchmark, rand_seed, variant);