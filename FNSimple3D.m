classdef FNSimple3D < handle
    properties (SetAccess = private)
        R                   % Radius of a wheel
        W                   % Width of the robot
        L                   % Length of the robot
        H                   % Height of the robot
        reach_cloud         % Stores reachable positions and orientations of the robot
        tree                % Array stores position information of states
        orientation         % Array stores orientation of a state of the robot
        manipulator         % Array stores angles of the manipulator links
        parent              % Array stores relations of nodes
        children            % Number of children of each node
        free_nodes          % Indices of free nodes
        free_nodes_ind      % Last element in free_nodes
        cost                % Cost between 2 connected states
        cumcost             % Cost from the root of the tree to the given node
        XY_BOUNDARY         % [min_x max_x min_y max_y]
        goal_point          % Goal position
        delta_goal_point    % Radius of goal position region
        delta_near          % Radius of near neighbor nodes
        nodes_added         % Keeps count of added nodes
        max_step            % The length of the maximum step while adding the node
        obstacle            % Obstacle information
        best_path_node      % The index of last node of the best path
        goal_reached
        %%% temporary variables
        compare_table
        index
        list
        num_rewired
    end
    methods
        % class constructor
        function this = FNSimple3D(rand_seed, max_nodes, map, conf)
            rng(rand_seed);
            this.R = conf.R;
            this.W = conf.W;
            this.L = conf.L;
            this.reach_cloud = zeros(3, 6561);
            this.tree = zeros(2, max_nodes);
            this.orientation = zeros(1, max_nodes);
            this.manipulator = zeros(5, max_nodes);
            this.parent = zeros(1, max_nodes);
            this.children = zeros(1, max_nodes);
            this.free_nodes = zeros(1, max_nodes);
            this.free_nodes_ind = 1;
            this.cost = zeros(1, max_nodes);
            this.cumcost = zeros(1,max_nodes);
            this.XY_BOUNDARY = zeros(4,1);
            this.tree(:, 1) = map.start_point; % Start position
            this.orientation(:, 1) = pi / 4; % Start orientation
            this.manipulator(:, 1) = zeros(5,1);
            this.goal_point = map.goal_point;
            this.delta_goal_point = conf.delta_goal_point;
            this.delta_near = conf.delta_near;
            this.nodes_added = uint32(1);
            this.max_step = conf.max_step;
            this.best_path_node = -1;
            this.goal_reached = false;
            this.load_map(map.name);
            %%% temp var-s initialization
            this.compare_table = zeros(1, max_nodes);
            this.index = zeros(1, max_nodes);
            this.list = 1:max_nodes;
            this.num_rewired = 0;
            this.init_reach_cloud();
        end
        
        function initialize = init_reach_cloud(this)
            count = 0;
            for t1 = -1:0.25:1
                for t2 = -1:0.25:1
                    for t3 = -1:0.25:1
                        for t4 = -1:0.25:1
                            count = count + 1;
                            this.reach_cloud(1, count) = this.R/4*(t1 + t2 + t3 + t4);
                            this.reach_cloud(2, count) = this.R/4*(-t1 + t2 + t3 - t4);
                            this.reach_cloud(3, count) = this.R/(4*(this.L + this.W))*(-t1 +t2 - t3 + t4);
                        end
                    end
                end
            end
            initialize = count;
        end
        
        function position = sample(this)
            % generates and return random point in area defined in
            % this.XY_BOUNDARY
            position = [this.XY_BOUNDARY(2) - this.XY_BOUNDARY(1); this.XY_BOUNDARY(4) - this.XY_BOUNDARY(3); pi] .* rand(3,1) ...
                + [this.XY_BOUNDARY(1);this.XY_BOUNDARY(3); 0];
            
        end
        
        function node_index = nearest(this, new_node)
            % find the nearest node to the given node, euclidian distance
            % is used
            this.compare_table(1:(this.nodes_added)) = sum((this.tree(:, 1:(this.nodes_added)) - repmat(new_node(1:2),1,this.nodes_added)).^2) + sum((this.orientation(1:this.nodes_added) - new_node(3)).^2);
            [this.compare_table(1:(this.nodes_added)), this.index(1:(this.nodes_added))] = sort(this.compare_table(1:(this.nodes_added)));
            node_index = this.index(1);
            return;
        end
        
        function position = steer(this, nearest_node, new_node_position)

            relative_position = new_node_position - [this.tree(:, nearest_node); this.orientation(nearest_node)];
            [min_val,  min_ind ] = min( sum( (this.reach_cloud - repmat(relative_position,1, length(this.reach_cloud)) ).^2));
            position = [this.tree(:, nearest_node); this.orientation(nearest_node)] + this.reach_cloud(:, min_ind);
            position(3) = mod(position(3) + 2*pi, 2*pi);
        end
        
        function load_map(this, map_name)
            % function loads '.mat' file with obstacle information and the
            % size of the map
            map_path = 'maps/';
            this.obstacle = load([map_path map_name], 'num', 'output', 'x_constraints', 'y_constraints');
            this.XY_BOUNDARY = [this.obstacle.x_constraints this.obstacle.y_constraints];
        end
        
        function collision = obstacle_collision(this, new_node_position, nearest_node)
            collision = false;
            theta = new_node_position(3) * 360 / pi;
            if (mod(theta, 90) == 0)
                theta = theta - 1;
            end
            % omit any operations if there is no obstacles
            if this.obstacle.num == 0
                return;
            end
            
            for obs_ind = 1:this.obstacle.num
                %if c_space_collided(this.obstacle.output{obs_ind}, new_node_position, theta) == 1
                % simple stupid collision detection based on line intersection
                
                vertex1 = [new_node_position(1) + cosd(theta)*this.L + sind(theta)*this.W new_node_position(2) + sind(theta)*this.L - cosd(theta)*this.W];
                vertex2 = [new_node_position(1) + cosd(theta)*this.L - sind(theta)*this.W new_node_position(2) + sind(theta)*this.L + cosd(theta)*this.W];
                vertex3 = [new_node_position(1) - cosd(theta)*this.L - sind(theta)*this.W new_node_position(2) - sind(theta)*this.L + cosd(theta)*this.W];
                vertex4 = [new_node_position(1) - cosd(theta)*this.L + sind(theta)*this.W new_node_position(2) - sind(theta)*this.L - cosd(theta)*this.W];
                
                if isintersect_3dof(this.obstacle.output{obs_ind}, [vertex1; vertex2]) == 1 ...
                        || isintersect_3dof(this.obstacle.output{obs_ind}, [vertex2; vertex3]) == 1 ...
                        || isintersect_3dof(this.obstacle.output{obs_ind}, [vertex3; vertex4]) == 1 ...
                        || isintersect_3dof(this.obstacle.output{obs_ind}, [vertex4; vertex1]) == 1
                    collision = true;
                    return;
                    %end
                end
            end
        end
        
        function new_node_ind = insert_node(this, parent_node_ind, new_node_position)
            % method insert new node in the tree
            this.nodes_added = this.nodes_added + 1;
            this.tree(:, this.nodes_added) = new_node_position(1:2);         % adding new node position to the tree
            this.orientation(this.nodes_added) = new_node_position(3);
            this.parent(this.nodes_added) = parent_node_ind;            % adding information about parent-children information
            this.children(parent_node_ind) = this.children(parent_node_ind) + 1;
            this.cost(this.nodes_added) = this.euclidian_distance([this.tree(:, parent_node_ind); this.orientation(parent_node_ind)], new_node_position);  % not that important
            this.cumcost(this.nodes_added) = this.cumcost(parent_node_ind) + this.cost(this.nodes_added);   % cummulative cost
            new_node_ind = this.nodes_added;
        end
        
        %%% RRT* specific functions
        
        function neighbor_nodes = neighbors(this, new_node_position, nearest_node_ind)
            % seeks for neighbors and returns indices of neighboring nodes
            radius = this.delta_near;
            this.compare_table(1:(this.nodes_added)) = sum((this.tree(:, 1:(this.nodes_added)) - repmat(new_node_position(1:2),1,this.nodes_added)).^2) + ...
                ((this.orientation(1:this.nodes_added) - new_node_position(3)).^2);
            [this.compare_table(1:(this.nodes_added)), this.index(1:(this.nodes_added))] = sort(this.compare_table(1:(this.nodes_added)));
            temp = this.index((this.compare_table(1:(this.nodes_added)) <= radius^2) & (this.compare_table(1:(this.nodes_added)) > 0 ));
            % neighbor_nodes = setdiff(temp, nearest_node_ind);
            neighbor_nodes = temp;
        end
        
        function min_node_ind = chooseParent(this, neighbors, nearest_node, new_node_position)
            % finds the node with minimal cummulative cost node from the root of
            % the tree. i.e. find the cheapest path end node.
            min_node_ind = nearest_node;
            min_cumcost = this.cumcost(nearest_node) + this.euclidian_distance([this.tree(:, nearest_node); this.orientation(nearest_node)], new_node_position);
            for ind=1:numel(neighbors)
                temp_cumcost = this.cumcost(neighbors(ind)) + this.euclidian_distance([this.tree(:, neighbors(ind)); this.orientation(neighbors(ind))], new_node_position);
                if temp_cumcost < min_cumcost
                    min_cumcost = temp_cumcost;
                    min_node_ind = neighbors(ind);
                end
            end
        end
        
        function rewire(this, new_node_ind, neighbors, min_node_ind)
            % method looks thru all neighbors(except min_node_ind) and
            % seeks and reconnects neighbors to the new node if it is
            % cheaper
            for ind = 1:numel(neighbors)
                % omit
                if (min_node_ind == neighbors(ind))
                    continue;
                end
                temp_cost = this.cumcost(new_node_ind) + this.euclidian_distance([this.tree(:, neighbors(ind)); this.orientation(neighbors(ind))], [this.tree(:, new_node_ind); this.orientation(new_node_ind)]);
                if (temp_cost < this.cumcost(neighbors(ind)))
                    this.cumcost(neighbors(ind)) = temp_cost;
                    this.children(this.parent(neighbors(ind))) = this.children(this.parent(neighbors(ind))) - 1;
                    this.parent(neighbors(ind)) = new_node_ind;
                    this.children(new_node_ind) = this.children(new_node_ind) + 1;
                    this.num_rewired = this.num_rewired + 1;
                end
            end
        end
        
        %%% RRT*FN specific functions
        
        function best_path_evaluate(this)
            %%% Find the optimal path to the goal
            % finding all the point which are in the desired region
            distances = zeros(this.nodes_added, 2);
            distances(:, 1) = sum((this.tree(:,1:(this.nodes_added)) - repmat(this.goal_point', 1, this.nodes_added)).^2);
            distances(:, 2) = 1:this.nodes_added;
            distances = sortrows(distances, 1);
            distances(:, 1) = distances(:, 1) <= (this.delta_goal_point ^ 2);
            dist_index = numel(find(distances(:, 1) == 1));
            % find the cheapest path
            if(dist_index ~= 0)
                distances(:, 1) = this.cumcost(int32(distances(:, 2)));
                distances = distances(1:dist_index, :);
                distances = sortrows(distances, 1);
                nearest_node_index = distances(1,2);
                this.goal_reached = true;
            else
                nearest_node_index = distances(1,2);
                if this.goal_reached
                    disp('VERYBAD THING HAS HAPPENED');
                end
                this.goal_reached = false;
            end
            %
            this.best_path_node = nearest_node_index;
        end
        
        function forced_removal(this)
            % removal function
            % we keep count of removed nodes
            candidate = this.list(this.children(1:(this.nodes_added)) == 0);
            node_to_remove = candidate(randi(numel(candidate)));
            while node_to_remove == this.best_path_node
                node_to_remove = candidate(randi(numel(candidate)));
            end
            this.children(this.parent(node_to_remove)) = this.children(this.parent(node_to_remove)) - 1;
            this.parent(node_to_remove) = -1;
            this.tree(:, node_to_remove) = [intmax; intmax];
            this.free_nodes(this.free_nodes_ind) = node_to_remove;
            this.free_nodes_ind = this.free_nodes_ind + 1;
        end
        
        function reused_node_ind = reuse_node(this, nearest_node, new_node_position)
            % method inserts new node instead of the removed one.
            if(this.free_nodes_ind == 1)
                disp('ERROR: Cannot find any free node!!!');
                return;
            end
            this.free_nodes_ind = this.free_nodes_ind - 1;
            reused_node_ind = this.free_nodes(this.free_nodes_ind);
            this.tree(:, reused_node_ind) = new_node_position(1:2);
            this.orientation(reused_node_ind) = new_node_position(3);
            this.parent(reused_node_ind) = nearest_node;
            this.children(nearest_node) = this.children(nearest_node) + 1;
            this.cost(reused_node_ind) = this.euclidian_distance([this.tree(:, nearest_node); this.orientation(nearest_node)], new_node_position);
            this.cumcost(reused_node_ind) = this.cumcost(nearest_node) + this.cost(reused_node_ind);
        end
        
        %%%%%%%%%%%%%%%%%%%%%%%%%
        function plot(this)
            %%% Find the optimal path to the goal
            % finding all the point which are in the desired region
            distances = zeros(this.nodes_added, 2);
            distances(:, 1) = sum((this.tree(:,1:(this.nodes_added)) - repmat(this.goal_point', 1, this.nodes_added)).^2);
            distances(:, 2) = 1:this.nodes_added;
            distances = sortrows(distances, 1);
            distances(:, 1) = distances(:, 1) <= this.delta_goal_point ^ 2;
            dist_index = numel(find(distances(:, 1) == 1));
            % find the cheapest path
            if(dist_index ~= 0)
                distances(:, 1) = this.cumcost(int32(distances(:, 2)));
                distances = distances(1:dist_index, :);
                distances = sortrows(distances, 1);
                nearest_node_index = distances(1,2);
            else
                disp('NOTICE! Robot cannot reach the goal');
                nearest_node_index = distances(1,2);
            end
            % backtracing the path
            current_index = nearest_node_index;
            path_iter = 1;
            backtrace_path = zeros(1,1);
            while(current_index ~= 1)
                backtrace_path(path_iter) = current_index;
                path_iter = path_iter + 1;
                current_index = this.parent(current_index);
            end
            backtrace_path(path_iter) = current_index;
            close all;
            figure;
            set(gcf(), 'Renderer', 'opengl');
            hold on;
            % obstacle drawing
            for k = 1:this.obstacle.num
                p2 = fill(this.obstacle.output{k}(1:end, 1), this.obstacle.output{k}(1:end, 2), 'r');
                set(p2,'HandleVisibility','off','EdgeAlpha',0);
            end
            
            drawn_nodes = zeros(1, this.nodes_added);
            for ind = this.nodes_added:-1:1;
                if(sum(this.free_nodes(1:this.free_nodes_ind) == ind)>0)
                    continue;
                end
                current_index = ind;
                while(current_index ~= 1 && current_index ~= -1)
                    % avoid drawing same nodes twice or more times
                    if(drawn_nodes(current_index) == false || drawn_nodes(this.parent(current_index)) == false)
                        plot([this.tree(1,current_index);this.tree(1, this.parent(current_index))], ...
                            [this.tree(2, current_index);this.tree(2, this.parent(current_index))],'g-','LineWidth', 0.5);
%                         plot([this.tree(1,current_index);this.tree(1, this.parent(current_index))], ...
%                             [this.tree(2, current_index);this.tree(2, this.parent(current_index))],'+k');
                        drawn_nodes(current_index) = true;
                        
                    end
                    current_index = this.parent(current_index);
                end
            end
            plot(this.tree(1,backtrace_path), this.tree(2,backtrace_path),'*b-','LineWidth', 2);
            
            plot([this.tree(1,backtrace_path) + cosd(this.orientation(backtrace_path)*360/pi)*this.L + sind(this.orientation(backtrace_path)*360/pi)*this.W; this.tree(1,backtrace_path) + cosd(this.orientation(backtrace_path)*360/pi)*this.L - sind(this.orientation(backtrace_path)*360/pi)*this.W; ...
                this.tree(1,backtrace_path) - cosd(this.orientation(backtrace_path)*360/pi)*this.L - sind(this.orientation(backtrace_path)*360/pi)*this.W;  this.tree(1,backtrace_path) - cosd(this.orientation(backtrace_path)*360/pi)*this.L + sind(this.orientation(backtrace_path)*360/pi)*this.W; ...
                this.tree(1,backtrace_path) + cosd(this.orientation(backtrace_path)*360/pi)*this.L + sind(this.orientation(backtrace_path)*360/pi)*this.W], ...
                [this.tree(2,backtrace_path) + sind(this.orientation(backtrace_path)*360/pi)*this.L - cosd(this.orientation(backtrace_path)*360/pi)*this.W; this.tree(2, backtrace_path) + sind(this.orientation(backtrace_path)*360/pi)*this.L + cosd(this.orientation(backtrace_path)*360/pi)*this.W; ...
                this.tree(2, backtrace_path) - sind(this.orientation(backtrace_path)*360/pi)*this.L + cosd(this.orientation(backtrace_path)*360/pi)*this.W; this.tree(2, backtrace_path) - sind(this.orientation(backtrace_path)*360/pi)*this.L - cosd(this.orientation(backtrace_path)*360/pi)*this.W; ...
                this.tree(2,backtrace_path) + sind(this.orientation(backtrace_path)*360/pi)*this.L - cosd(this.orientation(backtrace_path)*360/pi)*this.W], 'm', 'LineWidth', 2);
            
            this.plot_circle(this.goal_point(1), this.goal_point(2), this.delta_goal_point);
            axis(this.XY_BOUNDARY);
            grid on;
            axis equal;
            disp(num2str(this.cumcost(backtrace_path(1))));
        end
        
        function newObj = copyobj(thisObj)
            % Construct a new object based on a deep copy of the current
            % object of this class by copying properties over.
            props = properties(thisObj);
            for i = 1:length(props)
                % Use Dynamic Expressions to copy the required property.
                % For more info on usage of Dynamic Expressions, refer to
                % the section "Creating Field Names Dynamically" in:
                % web([docroot '/techdoc/matlab_prog/br04bw6-38.html#br1v5a9-1'])
                newObj.(props{i}) = thisObj.(props{i});
            end
        end
    end
    methods(Static)
        function dist = euclidian_distance(src_pos, dest_pos)
            dist = norm(src_pos - dest_pos);
        end
        function plot_circle(x, y, r)
            t = 0:0.001:2*pi;
            cir_x = r*cos(t) + x;
            cir_y = r*sin(t) + y;
            plot(cir_x, cir_y, 'r-', 'LineWidth', 1.5);
        end
    end
end
