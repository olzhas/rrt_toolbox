function problem = rrt(map, max_iter, is_benchmark, rand_seed, variant)
%RRT -- Rapidly-Exploring Random Tree is sampling-based algorithm, solves 
% the problem of motion and path planning providing feasible solutions
% however it does not take into account optimality of the solution.
%
% problem = RRT(map, max_iter, is_benchmark, rand_seed, variant)
% function returns the object of the respective class with the result
%
% map           -- struct with appropriate fields (developer of 
%               the class provides more information on this topic)
% max_iter      -- number of iteration to solve the problem
% is_benchmark  -- if true saves snapshots of the tree in a special directory
%               boolean variable
% rand_seed     -- a random seed 
% variant       -- what class to choose, class used defines the problem space
% 
%
% for detailed information consult with the help of the _RRT*FN Toolbox_
%
% Olzhas Adiyatov
% 05/13/2013

%%% Configuration block
if nargin < 5
    clear all;
    close all;
    clc;
    
    % loading conf
    RAND_SEED   = 1;
    MAX_ITER    = 20e3;
    MAX_NODES   = MAX_ITER;
    
    % here you can specify what class to use, each class represent
    % different model.
    % FNSimple2D provides RRT and RRT* for 2D mobile robot represented as a dot
    % FNRedundantManipulator represents redundant robotic manipulator, DOF is
    % defined in configuration files.
    
    
    variant     = 'FNSimple2D';
    MAP = struct('name', 'bench_june1.mat', 'start_point', [-12.5 -5.5], 'goal_point', [7 -3.65]);
    %     variant     = 'FNRedundantManipulator';
    %     MAP = struct('name', 'bench_redundant_3.mat', 'start_point', [0 0], 'goal_point', [35 35]);
    %     variant     = 'GridBased2Dimrotate';
    %     MAP = struct('name', 'grid_map.mat', 'start_point', [150 150], 'goal_point', [250 50]);
    
    
    % do we have to benchmark?
    is_benchmark = false;
else
    MAX_NODES   = max_iter;
    MAX_ITER    = max_iter;
    RAND_SEED   = rand_seed;
    MAP         = map;
end

addpath(genpath(pwd));

% loading settings
if exist(['configure_' variant '.m'], 'file')
    run([pwd '/configure_' variant '.m']);
    CONF = conf;
else
    disp('ERROR: There is no configuration file!')
    return
end

ALGORITHM = 'RRT';

problem = eval([variant '(RAND_SEED, MAX_NODES, MAP, CONF);']);

if(is_benchmark)
    benchmark_record_step = 250; % set step size of benchmark 250 by default
    benchmark_states = cell(MAX_ITER / benchmark_record_step, 1);
    timestamp = zeros(MAX_ITER / benchmark_record_step, 1);
    iterstamp = zeros(MAX_ITER / benchmark_record_step, 1);
end

%%% Starting a timer
tic;

%%% RRT starts here
for ind = 1:MAX_ITER
    new_node = problem.sample();
    nearest_node = problem.nearest(new_node);
    new_node = problem.steer(nearest_node, new_node);
    if(~problem.obstacle_collision(new_node, nearest_node))
        problem.insert_node(nearest_node, new_node);
    end
    
    if is_benchmark && (mod(ind, benchmark_record_step) == 0)
        benchmark_states{ind/benchmark_record_step} = problem.copyobj();
        timestamp(ind/benchmark_record_step) = toc;
        iterstamp(ind/benchmark_record_step) = ind;
    end
    
    if(mod(ind, 1000) == 0)
        disp([num2str(ind) ' iterations ' num2str(problem.nodes_added-1) ' nodes in ' num2str(toc)]);
    end
end


if (is_benchmark)
    if strcmp(computer, 'GLNXA64');
        result_dir = '/home/olzhas/june_results/';
    else
        result_dir = 'C:\june_results\';
    end
    dir_name = [result_dir datestr(now, 'yyyy-mm-dd')];
    mkdir(dir_name);
    save([dir_name '/' ALGORITHM '_' MAP.name '_' num2str(MAX_NODES) '_of_' num2str(MAX_ITER) '_' datestr(now, 'HH-MM-SS') '.mat'], '-v7.3');
    set(gcf, 'Visible', 'off');
    
    % free memory, sometimes there is a memory leak, or matlab decides to
    % free up memory later.
    clear all;
    clear('rrt.m');
    
    %     problem.plot();
    %     saveas(gcf, [dir_name '\' ALGORITHM '_' MAP.name '_' num2str(MAX_NODES) '_of_' num2str(MAX_ITER) '_' datestr(now, 'HH-MM-SS') '.fig']);
else
    problem.plot();
end