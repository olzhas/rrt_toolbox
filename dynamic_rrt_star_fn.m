function problem = dynamic_rrt_star_fn(map, max_iter, max_nodes, is_benchmark, rand_seed, variant)
%

%%% Configuration block
if nargin < 6
    clear all;
    close all;
    clc;
    
    % load conf
    RAND_SEED   = 1;
    MAX_ITER    = 30e3;
    MAX_NODES   = 4001;
    
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
    %[180 284]

    % do we have to benchmark?
    is_benchmark = false;
else
    MAX_NODES   = max_nodes;
    MAX_ITER    = max_iter;
    RAND_SEED   = rand_seed;
    MAP         = map;  
end

addpath(genpath(pwd));

%%% loading configuration file with object model specific options 
if exist(['configure_' variant '.m'], 'file')
    run([ pwd '/configure_' variant '.m']);
    CONF = conf;
else
    disp('ERROR: There is no configuration file!')
    return
end

ALGORITHM = 'RRTSFN';

problem = eval([variant '(RAND_SEED, MAX_NODES, MAP, CONF);']);
%problem = FNRedundantManipulator(RAND_SEED, MAX_NODES, MAP, CONF);


max_nodes_reached = false;

if(is_benchmark)
    benchmark_record_step = 250;
    benchmark_states = cell(MAX_ITER / benchmark_record_step, 1);
    
    timestamp = zeros(MAX_ITER / benchmark_record_step, 1);
    iterstamp = zeros(MAX_ITER / benchmark_record_step, 1);
end

%%% Starting a timer
tic;
for ind = 1:MAX_ITER
    
    if is_benchmark && max_nodes_reached == false && problem.nodes_added == MAX_NODES
        max_nodes_reached = true;
        before_removal_state = problem.copyobj();
    end
    
    problem.update_obstacles();
    
    new_node = problem.sample();
    nearest_node_ind = problem.nearest(new_node);
    new_node = problem.steer(nearest_node_ind, new_node); % if new node is very distant from the nearest node we go from the nearest node in the direction of a new node
    if(~problem.obstacle_collision(new_node, nearest_node_ind))
        neighbors = problem.neighbors(new_node, nearest_node_ind);
        if numel(neighbors) == 0 %this expression will show us that something is completely wrong
            %disp([' no neighbors detected at ' num2str(ind)]);
        end
        min_node = problem.chooseParent(neighbors, nearest_node_ind, new_node);
        if (problem.nodes_added == MAX_NODES)
            new_node_ind = problem.reuse_node(min_node, new_node);
        else
            new_node_ind = problem.insert_node(min_node, new_node);
        end
        problem.rewire(new_node_ind, neighbors, min_node);
        if (problem.nodes_added == MAX_NODES)
                problem.best_path_evaluate();
            problem.forced_removal();
        end
    end
    
    if is_benchmark && (mod(ind, benchmark_record_step) == 0)
        benchmark_states{ind/benchmark_record_step} = problem.copyobj();
        timestamp(ind/benchmark_record_step) = toc;
        iterstamp(ind/benchmark_record_step) = ind;
    end
    
    % display progress each 100 iterations
    if(mod(ind, 100) == 0)
        disp([num2str(ind) ' iterations ' num2str(problem.nodes_added-1) ' nodes in ' num2str(toc) ' rewired ' num2str(problem.num_rewired)]);
    end
end

if (is_benchmark)
    if strcmp(computer,'GLNXA64') == true
        result_dir = '/home/olzhas/june_results/';
    else
        result_dir = 'C:\june_results\';
    end
    dir_name = [result_dir datestr(now, 'yyyy-mm-dd')];
    mkdir(dir_name);
    save([dir_name '/' ALGORITHM '_' MAP.name '_' num2str(MAX_NODES) '_of_' num2str(MAX_ITER) '_' datestr(now, 'HH-MM-SS') '.mat']);
    set(gcf,'Visible','off');
%     problem.plot();
%     saveas(gcf, [dir_name '\' ALGORITHM '_' MAP.name '_' num2str(MAX_NODES) '_of_' num2str(MAX_ITER) '_' datestr(now, 'HH-MM-SS') '.fig']);
else
    problem.plot();
end

save('some.mat', '-v7.3');

% free memory
clear all;
clear('rrt_star_fn.m');

