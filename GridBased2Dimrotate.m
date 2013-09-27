classdef GridBased2Dimrotate < handle
    %
    % CUDA GPU required for this class
    %
    properties (SetAccess = private)
        tree                % Array stores position and angel information of states
        parent              % Array stores relations of nodes
        children            % Number of children of each node
        free_nodes          % Indices of free nodes
        free_nodes_ind      % Last element in free_nodes
        cost                % Cost between 2 connected states
        cumcost             % Cost from the root of the tree to the given node
        XY_BOUNDARY         % [min_x max_x min_y max_y]
        SCALED_XY_BOUNDARY
        goal_point          % Goal position
        delta_goal_point    % Radius of goal position region
        delta_near          % Radius of near neighbor nodes
        nodes_added         % Keeps count of added nodes
        max_step            % The length of the maximum step while adding the node
        best_path_node      % The index of last node of the best path
        map                 % grid based map
        map_gpu
        scaled_map
        scaled_map_gpu
        goal_reached
        half_length
        half_width
        turn_theta
        rob_rect
        rob_rect_gpu
        scaled_rob_rect
        scaled_rob_rect_gpu
        %%% temporary variables
        compare_table
        index
        list
        num_rewired
        radius
    end
    methods
        % class constructor
        function this = GridBased2Dimrotate(rand_seed, max_nodes, map, conf)
            rng(rand_seed);
            this.tree = zeros(3, max_nodes);
            this.parent = zeros(1, max_nodes);
            this.children = zeros(1, max_nodes);
            this.free_nodes = zeros(1, max_nodes);
            this.free_nodes_ind = 1;
            this.cost = zeros(1, max_nodes);
            this.cumcost = zeros(1,max_nodes);
            this.XY_BOUNDARY = zeros(4,1);
            this.tree(1:2, 1) = map.start_point;
            this.goal_point = map.goal_point;
            this.delta_goal_point = conf.delta_goal_point;
            this.delta_near = conf.delta_near;
            this.nodes_added = uint32(1);
            this.max_step = conf.max_step;
            this.best_path_node = -1;
            this.goal_reached = false;
            this.half_width = conf.half_width;
            this.half_length = conf.half_length;
            this.turn_theta = conf.turn_theta;
            this.load_map(map.name);
            this.rob_rect = ones(2*this.half_length, 2*this.half_width, 'uint8');
            this.rob_rect_gpu = gpuArray(this.rob_rect);
            this.scaled_rob_rect = ones(ceil(this.half_length/2), ceil(this.half_width/2), 'uint8');
            this.scaled_rob_rect_gpu = gpuArray(this.scaled_rob_rect);
            %%% temp var-s initialization
            this.compare_table = zeros(1, max_nodes);
            this.index = zeros(1, max_nodes);
            this.list = 1:max_nodes;
            this.num_rewired = 0;
            this.radius = sqrt(this.delta_near^2+this.turn_theta^2);
        end
        
        function position = sample(this)
            % generates and return random point in area defined in
            % this.XY_BOUNDARY
            position = zeros(3,1);
            position(1:2) = int32([this.XY_BOUNDARY(2) - this.XY_BOUNDARY(1); this.XY_BOUNDARY(4) - this.XY_BOUNDARY(3)] .* rand(2,1) ...
                + [this.XY_BOUNDARY(1);this.XY_BOUNDARY(3)]);
            position(3) = 360*rand();
        end
        
        function node_index = nearest(this, new_node)
            if size(new_node,1) ~= 3
                new_node = new_node';
            end
            % find the nearest node to the given node, euclidian distance
            % is used
            this.compare_table(1:(this.nodes_added)) = sum((this.tree(:, 1:(this.nodes_added)) - repmat(new_node(:),1,this.nodes_added)).^2);
            
            [this.compare_table(1:(this.nodes_added)), this.index(1:(this.nodes_added))] = sort(this.compare_table(1:(this.nodes_added)));
            node_index = this.index(1);
            return;
        end
        
        function position = steer(this, nearest_node, new_node_position)
            position = zeros(3,1);
            % if new node is very distant from the nearest node we go from the nearest node in the direction of a new node
            if(this.euclidian_distance(new_node_position(1:2), this.tree(1:2, nearest_node)) > this.max_step)
                theta = atan((new_node_position(2) - this.tree(2, nearest_node))/(new_node_position(1) - this.tree(1, nearest_node)));
                position(1:2) = round(this.tree(1:2, nearest_node) ...
                    + [sign((new_node_position(1) - this.tree(1, nearest_node))) * this.max_step * cos(theta); ...
                    sign((new_node_position(2) - this.tree(2, nearest_node))) * this.max_step * abs(sin(theta))]);
            else
                position(1:2) = new_node_position(1:2);
            end
            if abs(new_node_position(3) - this.tree(3, nearest_node)) > this.turn_theta
                position(3) = this.tree(3, nearest_node) + sign(new_node_position(3) - this.tree(3, nearest_node))*this.turn_theta;
            else
                position(3) = new_node_position(3);
            end
        end
        
        function load_map(this, map_name)
            % function loads '.mat' file with obstacle information and the
            % size of the map
            map_path = 'maps/';
            load([map_path map_name], 'my_map');
            this.map = uint8(my_map);
            this.map_gpu = gpuArray(this.map);
            this.scaled_map = zeros(size(this.map,1),size(this.map,2),'uint8');
            for ind1 = 1:round(size(this.map,1)/4)
                for ind2 = 1:round(size(this.map,2)/4)
                    this.scaled_map(ind1,ind2) = any(any(this.map((ind1*4-3):ind1*4, (ind2*4-3):ind2*4)));
                end
            end
            this.scaled_map_gpu = gpuArray(this.scaled_map);
            this.XY_BOUNDARY = [1 size(this.map,1) 1 size(this.map,2)];
            this.SCALED_XY_BOUNDARY = [1 round(size(this.map,1)/4) 1 round(size(this.map,2)/4)];
        end
        
        function collision = obstacle_collision(this, new_node_position, node_index)
            collision = false;
            
            %% first check for scaled model of robot 1/4 of map size
            rob = imrotate(this.scaled_rob_rect_gpu, new_node_position(3));
            temp1 = int32(round(size(rob,1)/2)-[0 mod(size(rob,1),2)]);
            temp2 = int32(round(size(rob,2)/2)-[0 mod(size(rob,2),2)]);
            x_pos = [floor(new_node_position(1)/4)-temp1(1)+1; floor(new_node_position(1)/4) + temp1(2)];
            y_pos = [floor(new_node_position(2)/4)-temp2(1)+1; floor(new_node_position(2)/4) + temp2(2)];
            if any(x_pos > this.SCALED_XY_BOUNDARY(2)) || any(x_pos < this.SCALED_XY_BOUNDARY(1)) || ...
                    any(y_pos > this.SCALED_XY_BOUNDARY(4)) || any(y_pos < this.SCALED_XY_BOUNDARY(3))
                collision = true;
                return;
            end
            
            diff_mat = this.scaled_map_gpu(y_pos(1):y_pos(2), x_pos(1):x_pos(2))' + uint8(rob);
            if any(diff_mat(:) > 1)
                %                 collision = true;
                %                 return;
                %% collision detected further tests
                rob = imrotate(this.rob_rect_gpu, new_node_position(3));
                temp1 = int32(round(size(rob,1)/2)-[0 mod(size(rob,1),2)]);
                temp2 = int32(round(size(rob,2)/2)-[0 mod(size(rob,2),2)]);
                x_pos = [floor(new_node_position(1)/4)-temp1(1)+1; floor(new_node_position(1)/4) + temp1(2)];
                y_pos = [floor(new_node_position(2)/4)-temp2(1)+1; floor(new_node_position(2)/4) + temp2(2)];
                if any(x_pos > this.SCALED_XY_BOUNDARY(2)) || any(x_pos < this.SCALED_XY_BOUNDARY(1)) || ...
                        any(y_pos > this.SCALED_XY_BOUNDARY(4)) || any(y_pos < this.SCALED_XY_BOUNDARY(3))
                    collision = true;
                    return;
                end
                diff_mat = this.map_gpu(y_pos(1):y_pos(2), x_pos(1):x_pos(2))' + uint8(rob);
                if any(diff_mat(:) > 1)
                    collision = true;
                    return;
                end
            end
        end
        
        function new_node_ind = insert_node(this, parent_node_ind, new_node_position)
            % method insert new node in the tree
            this.nodes_added = this.nodes_added + 1;
            this.tree(:, this.nodes_added) = new_node_position;         % adding new node position to the tree
            this.parent(this.nodes_added) = parent_node_ind;            % adding information about parent-children information
            this.children(parent_node_ind) = this.children(parent_node_ind) + 1;
            this.cost(this.nodes_added) = this.euclidian_distance(this.tree(:, parent_node_ind), new_node_position);  % not that important
            this.cumcost(this.nodes_added) = this.cumcost(parent_node_ind) + this.cost(this.nodes_added);   % cummulative cost
            new_node_ind = this.nodes_added;
        end
        
        %%% RRT* specific functions
        
        function neighbor_nodes = neighbors(this, new_node_position, nearest_node_ind)
            
            if size(new_node_position,1) ~= 3
                new_node_position = new_node_position';
            end
            % seeks for neighbors and returns indices of neighboring nodes
            
            this.compare_table(1:(this.nodes_added)) = sum((this.tree(:, 1:(this.nodes_added)) - repmat(new_node_position(:),1,this.nodes_added)).^2);
            [this.compare_table(1:(this.nodes_added)), this.index(1:(this.nodes_added))] = sort(this.compare_table(1:(this.nodes_added)));
            temp = this.index((this.compare_table(1:(this.nodes_added)) <= this.radius^2) & (this.compare_table(1:(this.nodes_added)) > 0 ));
            neighbor_nodes = temp;
            %neighbor_nodes = setdiff(temp, nearest_node_ind);
        end
        
        function min_node_ind = chooseParent(this, neighbors, nearest_node, new_node_position)
            % finds the node with minimal cummulative cost node from the root of
            % the tree. i.e. find the cheapest path end node.
            min_node_ind = nearest_node;
            min_cumcost = this.cumcost(nearest_node) + this.euclidian_distance(this.tree(:, nearest_node), new_node_position);
            for ind=1:numel(neighbors)
                
                if(~this.obstacle_collision(new_node_position, neighbors(ind)))
                    temp_cumcost = this.cumcost(neighbors(ind)) + this.euclidian_distance(this.tree(:, neighbors(ind)), new_node_position);
                    if temp_cumcost < min_cumcost
                        min_cumcost = temp_cumcost;
                        min_node_ind = neighbors(ind);
                    end
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
                temp_cost = this.cumcost(new_node_ind) + this.euclidian_distance(this.tree(:, neighbors(ind)), this.tree(:, new_node_ind));
                if temp_cost < this.cumcost(neighbors(ind)) %&& ...
                    %~this.obstacle_collision(this.tree(:, new_node_ind), neighbors(ind))
                    
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
            distances(:, 1) = sum((this.tree(1:2,1:(this.nodes_added)) - repmat(this.goal_point', 1, this.nodes_added)).^2);
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
                    disp('VERYBAD THINGHAS HAPPENED');
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
                disp('attempt to delete last node in the feasible path');
            end
            this.children(this.parent(node_to_remove)) = this.children(this.parent(node_to_remove)) - 1;
            this.parent(node_to_remove) = -1;
            this.tree(1:2, node_to_remove) = [intmax; intmax];
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
            this.tree(:, reused_node_ind) = new_node_position;
            this.parent(reused_node_ind) = nearest_node;
            this.children(nearest_node) = this.children(nearest_node) + 1;
            this.cost(reused_node_ind) = this.euclidian_distance(this.tree(:, nearest_node), new_node_position);
            this.cumcost(reused_node_ind) = this.cumcost(nearest_node) + this.cost(reused_node_ind);
        end
        
        %%%%%%%%%%%%%%%%%%%%%%%%%
        function plot(this)
            %%% Find the optimal path to the goal
            % finding all the point which are in the desired region
            distances = zeros(this.nodes_added, 2);
            distances(:, 1) = sum((this.tree(1:2,1:(this.nodes_added)) - repmat(this.goal_point', 1, this.nodes_added)).^2);
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
            set(gcf, 'Position', [0 0 2000 1000]);
            subplot(1,2,1);
            imshow(this.map*50);
            set(gcf(), 'Renderer', 'opengl');
            hold on;
            
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
                        plot([this.tree(1,current_index);this.tree(1, this.parent(current_index))], ...
                            [this.tree(2, current_index);this.tree(2, this.parent(current_index))],'.c');
                        drawn_nodes(current_index) = true;
                    end
                    current_index = this.parent(current_index);
                end
            end
            plot(this.tree(1,backtrace_path), this.tree(2,backtrace_path),'*b-','LineWidth', 2);
            this.plot_circle(this.goal_point(1), this.goal_point(2), this.delta_goal_point);
            axis(this.XY_BOUNDARY);
            axis on;
            grid on;
            box on;
            subplot(1,2,2);
            imshow(this.map*50);
            set(gcf(), 'Renderer', 'opengl');
            hold on;
            for ind=numel(backtrace_path):-1:1
                subplot(1,2,2);
                rob = imrotate(this.rob_rect, this.tree(3,backtrace_path(ind)));
                temp1 = int32(floor(size(rob,1)/2));
                temp2 = int32(floor(size(rob,2)/2));
                x_pos = [this.tree(1,backtrace_path(ind))-temp1; this.tree(1,backtrace_path(ind)) + temp1];
                y_pos = [this.tree(2,backtrace_path(ind))-temp2; this.tree(2,backtrace_path(ind)) + temp2];
                temp_map = this.map;
                if(numel(y_pos(1):y_pos(2)) - size(rob,2) == 1) && (numel(x_pos(1):x_pos(2)) - size(rob,1) == 1)
                    rob = [rob zeros(numel(x_pos(1):x_pos(2))-1,1)];
                    rob = [rob ;zeros(1,numel(y_pos(1):y_pos(2)))];
                end
                temp_map(y_pos(1):y_pos(2), x_pos(1):x_pos(2)) = rob'*2;
                imshow(temp_map*50);
                pause(0.1);
                drawnow;
            end
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