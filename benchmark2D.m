% benchmark script for 2D mobile robot
% Olzhas Adiyatov
% 6/4/2013
%

clear;
close all;
clc;
addpath(genpath(pwd));

maps = cell(3,1);
maps_name = {'bench_june1.mat', 'bench_june2.mat', 'bench_june3.mat'};
goal_pos = {[7 -3.65], [4.8 -12], [9, -6]};
init_conf = {[-12.5 -5.5], [-10.5, 7], [0 15]};

for ind = 1:3
    maps{ind} = struct('name', maps_name{ind}, 'start_point', init_conf{ind}, 'goal_point', goal_pos{ind});
end

max_iter = 20e3;
max_nodes = [500 1000 2000 4000];
rand_seed = [10 12 1337 2971 9999 87 645 287 59 7777];

cl = parcluster;
for rand_ind = 3:numel(rand_seed)
    for map_ind = 1:3
        batch('rrt', 0,{maps{map_ind}, max_iter, true, rand_seed(rand_ind), 'FNSimple2D'});
        for mn_ind = 1:numel(max_nodes)
            batch('rrt_star_fn', 0,{maps{map_ind}, max_iter, max_nodes(mn_ind), true, rand_seed(rand_ind), 'FNSimple2D'});
        end
        
    end
    
    j1 = batch('rrt_star', 0,{maps{1}, max_iter, true, rand_seed(rand_ind), 'FNSimple2D'});
    j2 = batch('rrt_star', 0,{maps{2}, max_iter, true, rand_seed(rand_ind), 'FNSimple2D'});
    j3 = batch('rrt_star', 0,{maps{3}, max_iter, true, rand_seed(rand_ind), 'FNSimple2D'});
    wait(j1); wait(j2); wait(j3);
    delete(cl.Jobs);
        
end
