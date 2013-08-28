% benchmark script for nDOF redundant planar manipulator
% Olzhas Adiyatov
% 6/20/2013
%

% clear all variables, class, console, close all figures
clear all;
close all;
clc;
addpath(genpath(pwd));  % add current directory recursively to the MATLAB path

maps = cell(3,1);
maps_name = {'bench_redundant_1.mat', 'bench_redundant_2.mat', 'bench_redundant_3.mat', 'null.mat'};
goal_pos = {[-20 40], [-30 5], [35 35], [20 10]};
init_conf = {[0 0], [0 0], [0 0], [0 0]};

for ind = 1:3
    maps{ind} = struct('name', maps_name{ind}, 'start_point', init_conf{ind}, 'goal_point', goal_pos{ind});
end

max_iter = 30e3;
max_nodes = [500 1000 2000 4000];
rand_seed = 1488;

for map_ind = 1:3
    rrt_star(maps{map_ind}, max_iter, true, rand_seed, 'FNRedundantManipulator');
    rrt(maps{map_ind}, max_iter, true, rand_seed, 'FNRedundantManipulator');
    
    for mn_ind = 1:numel(max_nodes)
        rrt_star_fn(maps{map_ind}, max_iter, max_nodes(mn_ind), true, rand_seed, 'FNRedundantManipulator'); 
    end
end
