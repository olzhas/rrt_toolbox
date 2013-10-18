classdef FNSimple2D < handle
    properties (SetAccess = private)
        tree                % Array stores position information of states
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
        dynamic_obstacle    % Dynamic Obstacles Information
        best_path_node      % The index of last node of the best path
        goal_reached
        max_nodes 
        %%% Binning for faster neighbor search
        % bins are square
        bin_size
        bin
        bin_x
        bin_y
        bin_offset
        nbins
        bin_ind
        %%% temporary variables
        compare_table
        index
        list
        num_rewired
    end
    methods
        % class constructor
        function this = FNSimple2D(rand_seed, max_nodes, map, conf)
            max_nodes = int32(max_nodes);
            this.max_nodes = max_nodes;
            rng(rand_seed);
            this.tree = zeros(2, max_nodes);
            this.parent = zeros(1, max_nodes);
            this.children = zeros(1, max_nodes);
            this.free_nodes = zeros(1, max_nodes);
            this.free_nodes_ind = 1;
            this.cost = zeros(1, max_nodes);
            this.cumcost = zeros(1,max_nodes);
            this.XY_BOUNDARY = zeros(4,1);
            this.tree(:, 1) = map.start_point;
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
            
            this.bin_ind = zeros(10, max_nodes);
            
            this.bin_size = conf.bin_size;
            this.bin_x = ceil((this.XY_BOUNDARY(2) - this.XY_BOUNDARY(1))/this.bin_size);
            this.bin_y = ceil((this.XY_BOUNDARY(4) - this.XY_BOUNDARY(2))/this.bin_size);
            
            delta = this.bin_size/100;
            left_edge = int32((this.XY_BOUNDARY(1) + delta) / this.bin_size - 0.5);
            bottom_edge = int32((this.XY_BOUNDARY(3) + delta) / this.bin_size - 0.5);
            right_edge = int32((this.XY_BOUNDARY(2) - delta) / this.bin_size - 0.5);
            top_edge = int32((this.XY_BOUNDARY(4) - delta) / this.bin_size - 0.5);
            
            this.bin_offset = -(left_edge + bottom_edge*this.bin_x) + 1;
            this.nbins = (right_edge + top_edge*this.bin_x) - (left_edge + bottom_edge*this.bin_x)+ 1;
            this.bin = repmat(struct('nodes', zeros(1, int32(max_nodes/5)), 'last', 0), 1, this.nbins);
            
            % add root node into bin
            x_comp = int32(map.start_point(1) / this.bin_size - 0.5);
            y_comp = int32(map.start_point(2) / this.bin_size - 0.5);
            cur_bin = x_comp + y_comp*this.bin_x + this.bin_offset;
            this.bin(cur_bin).last = this.bin(cur_bin).last + 1;
            this.bin(cur_bin).nodes(this.bin(cur_bin).last) = 1;
            this.bin_ind(end,1) = this.bin_ind(end, 1) + 1;
            this.bin_ind(this.bin_ind(end,1),1) = cur_bin;
            
            %% placing nodes in additional bins
            radius = this.delta_near;
            x_left = x_comp;
            x_right = x_comp;
            y_top = y_comp;
            y_bottom = y_comp;
            if map.start_point(1) - radius >= this.XY_BOUNDARY(1)
                x_left = int32((map.start_point(1) - radius)/this.bin_size - 0.5);
            end
            if map.start_point(1) + radius <= this.XY_BOUNDARY(2)
                x_right = int32((map.start_point(1) + radius)/this.bin_size - 0.5);
            end
            if map.start_point(2) - radius >= this.XY_BOUNDARY(3)
                y_top = int32((map.start_point(2) + radius)/this.bin_size - 0.5);
            end
            if map.start_point(2) + radius <= this.XY_BOUNDARY(4)
                y_bottom = int32((map.start_point(2) - radius)/this.bin_size - 0.5);
            end
            
            if x_comp > x_left && cur_bin - 1 > 0
                this.bin(cur_bin-1).last = this.bin(cur_bin-1).last + 1;
                this.bin(cur_bin-1).nodes(this.bin(cur_bin-1).last) = 1;
                
                this.bin_ind(end,1) = this.bin_ind(end, 1) + 1;
                this.bin_ind(this.bin_ind(end,1),1) = cur_bin-1;
            end
            
            if x_comp < x_right && cur_bin + 1 < this.nbins
                this.bin(cur_bin+1).last = this.bin(cur_bin+1).last + 1;
                this.bin(cur_bin+1).nodes(this.bin(cur_bin+1).last) = 1;
                
                this.bin_ind(end,1) = this.bin_ind(end, 1) + 1;
                this.bin_ind(this.bin_ind(end,1),1) = cur_bin+1;
            end
            
            if y_comp < y_top
                if cur_bin+this.bin_x <= this.nbins
                    this.bin(cur_bin+this.bin_x).last = this.bin(cur_bin+this.bin_x).last + 1;
                    this.bin(cur_bin+this.bin_x).nodes(this.bin(cur_bin+this.bin_x).last) = 1;
                    
                    this.bin_ind(end,1) = this.bin_ind(end, 1) + 1;
                    this.bin_ind(this.bin_ind(end,1),1) = cur_bin+this.bin_x;
                    if x_comp > x_left
                        this.bin(cur_bin-1+this.bin_x).last = this.bin(cur_bin-1+this.bin_x).last + 1;
                        this.bin(cur_bin-1+this.bin_x).nodes(this.bin(cur_bin-1+this.bin_x).last) = 1;
                        
                        this.bin_ind(end,1) = this.bin_ind(end, 1) + 1;
                        this.bin_ind(this.bin_ind(end,1),1) = cur_bin-1+this.bin_x;
                    end
                    if x_comp < x_right && cur_bin+this.bin_x+1 <= this.nbins
                        this.bin(cur_bin+1+this.bin_x).last = this.bin(cur_bin+1+this.bin_x).last + 1;
                        this.bin(cur_bin+1+this.bin_x).nodes(this.bin(cur_bin+1+this.bin_x).last) = 1;
                        
                        this.bin_ind(end,1) = this.bin_ind(end, 1) + 1;
                        this.bin_ind(this.bin_ind(end,1),1) = cur_bin+1+this.bin_x;
                    end
                end
            end
            
            if y_comp > y_bottom
                if cur_bin-this.bin_x > 0
                    this.bin(cur_bin-this.bin_x).last = this.bin(cur_bin-this.bin_x).last + 1;
                    this.bin(cur_bin-this.bin_x).nodes(this.bin(cur_bin-this.bin_x).last) = 1;
                    
                    this.bin_ind(end,1) = this.bin_ind(end, 1) + 1;
                    this.bin_ind(this.bin_ind(end,1),1) = cur_bin-this.bin_x;
                    
                    if x_comp > x_left && cur_bin-1-this.bin_x > 0
                        this.bin(cur_bin-1-this.bin_x).last = this.bin(cur_bin-1-this.bin_x).last + 1;
                        this.bin(cur_bin-1-this.bin_x).nodes(this.bin(cur_bin-1-this.bin_x).last) = 1;
                        
                        this.bin_ind(end,1) = this.bin_ind(end, 1) + 1;
                        this.bin_ind(this.bin_ind(end,1),1) = cur_bin-1-this.bin_x;
                    end
                    if x_comp < x_right
                        this.bin(cur_bin+1-this.bin_x).last = this.bin(cur_bin+1-this.bin_x).last + 1;
                        this.bin(cur_bin+1-this.bin_x).nodes(this.bin(cur_bin+1-this.bin_x).last) = 1;
                        
                        this.bin_ind(end,1) = this.bin_ind(end, 1) + 1;
                        this.bin_ind(this.bin_ind(end,1),1) = cur_bin+1-this.bin_x;
                    end
                end
            end
        end
        
        function position = sample(this)
            % generates and return random point in area defined in
            % this.XY_BOUNDARY
            position = [this.XY_BOUNDARY(2) - this.XY_BOUNDARY(1); this.XY_BOUNDARY(4) - this.XY_BOUNDARY(3)] .* rand(2,1) ...
                + [this.XY_BOUNDARY(1);this.XY_BOUNDARY(3)];
        end
        
        function node_index = nearest(this, new_node_position)
            radius = this.delta_near;
            % % binning chunk of code
            x_comp = int32(new_node_position(1) / this.bin_size - 0.5);
            y_comp = int32(new_node_position(2) / this.bin_size - 0.5);
            
            cur_bin = x_comp + y_comp*this.bin_x + this.bin_offset;
            
            num_nbors = this.bin(cur_bin).last;
            
            if num_nbors < 20
                nbors = 1:this.nodes_added;
                num_nbors = this.nodes_added;
            else
                nbors = this.bin(cur_bin).nodes(1:num_nbors);
            end
            
            this.compare_table(1:num_nbors) = sum((this.tree(:, nbors) - repmat(new_node_position,1,num_nbors)).^2);
            % just in case
            if this.free_nodes_ind > 1
                this.compare_table(this.free_nodes(1:(this.free_nodes_ind-1))) = intmax;
            end
            [this.compare_table(1:num_nbors), this.index(1:num_nbors)] = min(this.compare_table(1:num_nbors));
            temp = nbors(this.index((this.compare_table(1:num_nbors) <= radius^2*1.00001) & (this.compare_table(1:num_nbors) > 0 )));
            
            node_index = nbors(this.index(1));
        end
        
        function position = steer(this, nearest_node, new_node_position)
            % if new node is very distant from the nearest node we go from the nearest node in the direction of a new node
            if(norm(new_node_position - this.tree(:, nearest_node)) > this.max_step)
                theta = atan((new_node_position(2) - this.tree(2, nearest_node))/(new_node_position(1) - this.tree(1, nearest_node)));
                position = this.tree(:, nearest_node) ...
                    + [sign((new_node_position(1) - this.tree(1, nearest_node))) * this.max_step * cos(theta); ...
                    sign((new_node_position(2) - this.tree(2, nearest_node))) * this.max_step * abs(sin(theta))];
            else
                position = new_node_position;
            end
        end
        
        function load_map(this, map_name)
            % function loads '.mat' file with obstacle information and the
            % size of the map
            map_path = 'maps/';
            this.obstacle = load([map_path map_name], 'num', 'output', 'x_constraints', 'y_constraints');
            this.obstacle.vert_num = zeros(this.obstacle.num,1);
            this.obstacle.m = cell(this.obstacle.num,1);
            this.obstacle.b = cell(this.obstacle.num,1);
            this.obstacle.r = zeros(this.obstacle.num,1);
            this.obstacle.r_sqr = zeros(this.obstacle.num,1);
            this.obstacle.cir_center = cell(this.obstacle.num,1);
            
            
            this.XY_BOUNDARY = [this.obstacle.x_constraints this.obstacle.y_constraints];
            for obs_ind = 1:this.obstacle.num
                this.obstacle.m{obs_ind} = (this.obstacle.output{obs_ind}(1:end-1,2) - this.obstacle.output{obs_ind}(2:end,2)) ./ (this.obstacle.output{obs_ind}(1:end-1,1) - this.obstacle.output{obs_ind}(2:end,1));
                this.obstacle.b{obs_ind} = this.obstacle.output{obs_ind}(1:end-1,2) - this.obstacle.m{obs_ind} .* this.obstacle.output{obs_ind}(1:end-1,1);
                this.obstacle.vert_num(obs_ind) = size(this.obstacle.output{obs_ind}, 1)-1;
                
                this.obstacle.cir_center{obs_ind} = zeros(2, 1);
                [this.obstacle.cir_center{obs_ind}(1), this.obstacle.cir_center{obs_ind}(2), this.obstacle.r(obs_ind)] = ...
                    SmallestEnclosingCircle(this.obstacle.output{obs_ind}(1:end-1,1)', this.obstacle.output{obs_ind}(1:end-1,2)');
                this.obstacle.r_sqr(obs_ind) = this.obstacle.r(obs_ind) ^ 2;
            end
        end
        
        function collision = obstacle_collision(this, new_node_position, node_index)
            collision = false;
            % omit any operations if there is no obstacles
            if this.obstacle.num == 0
                return;
            end
            
            for obs_ind = 1:this.obstacle.num
                % circle as a bounding shape test
                if sum((this.obstacle.cir_center{obs_ind} - new_node_position) .^2) <= this.obstacle.r_sqr(obs_ind) || ...
                        sum((this.obstacle.cir_center{obs_ind} - this.tree(:,node_index)) .^2) <= this.obstacle.r_sqr(obs_ind)
                    % simple stupid collision detection based on line intersection
                    if isintersect(this.obstacle.output{obs_ind}, [this.tree(:, node_index) new_node_position]', ...
                            this.obstacle.m{obs_ind}, this.obstacle.b{obs_ind}, this.obstacle.vert_num(obs_ind)) == 1
                        collision = true;
                        return;
                    end
                end
            end
        end
        
        function new_node_ind = insert_node(this, parent_node_ind, new_node_position)
            % method insert new node in the tree
            this.nodes_added = this.nodes_added + 1;
            this.tree(:, this.nodes_added) = new_node_position;         % adding new node position to the tree
            this.parent(this.nodes_added) = parent_node_ind;            % adding information about parent-children information
            this.children(parent_node_ind) = this.children(parent_node_ind) + 1;
            this.cost(this.nodes_added) = norm(this.tree(:, parent_node_ind) - new_node_position);  % not that important
            this.cumcost(this.nodes_added) = this.cumcost(parent_node_ind) + this.cost(this.nodes_added);   % cummulative cost
            new_node_ind = this.nodes_added;
            
            radius = this.delta_near;
            
            x_comp = int32(new_node_position(1) / this.bin_size - 0.5);
            y_comp = int32(new_node_position(2) / this.bin_size - 0.5);
            
            cur_bin = x_comp + y_comp*this.bin_x + this.bin_offset;
            
            this.bin(cur_bin).last = this.bin(cur_bin).last + 1;
            this.bin(cur_bin).nodes(this.bin(cur_bin).last) = new_node_ind;
            
            this.bin_ind(end,new_node_ind) = this.bin_ind(end, new_node_ind) + 1;
            this.bin_ind(this.bin_ind(end,new_node_ind),new_node_ind) = cur_bin;
            
            %% placing nodes in additional bins
            x_left = x_comp;
            x_right = x_comp;
            y_top = y_comp;
            y_bottom = y_comp;
            if new_node_position(1) - radius >= this.XY_BOUNDARY(1)
                x_left = int32((new_node_position(1) - radius)/this.bin_size - 0.5);
            end
            if new_node_position(1) + radius <= this.XY_BOUNDARY(2)
                x_right = int32((new_node_position(1) + radius)/this.bin_size - 0.5);
            end
            if new_node_position(2) - radius >= this.XY_BOUNDARY(3)
                y_top = int32((new_node_position(2) + radius)/this.bin_size - 0.5);
            end
            if new_node_position(2) + radius <= this.XY_BOUNDARY(4)
                y_bottom = int32((new_node_position(2) - radius)/this.bin_size - 0.5);
            end
            
            if x_comp > x_left && cur_bin - 1 > 0
                this.bin(cur_bin-1).last = this.bin(cur_bin-1).last + 1;
                this.bin(cur_bin-1).nodes(this.bin(cur_bin-1).last) = new_node_ind;
                
                this.bin_ind(end,new_node_ind) = this.bin_ind(end, new_node_ind) + 1;
                this.bin_ind(this.bin_ind(end,new_node_ind),new_node_ind) = cur_bin-1;
            end
            
            if x_comp < x_right && cur_bin + 1 < this.nbins
                this.bin(cur_bin+1).last = this.bin(cur_bin+1).last + 1;
                this.bin(cur_bin+1).nodes(this.bin(cur_bin+1).last) = new_node_ind;
                
                this.bin_ind(end,new_node_ind) = this.bin_ind(end, new_node_ind) + 1;
                this.bin_ind(this.bin_ind(end,new_node_ind),new_node_ind) = cur_bin+1;
            end
            
            if y_comp < y_top
                if cur_bin+this.bin_x <= this.nbins
                    this.bin(cur_bin+this.bin_x).last = this.bin(cur_bin+this.bin_x).last + 1;
                    this.bin(cur_bin+this.bin_x).nodes(this.bin(cur_bin+this.bin_x).last) = new_node_ind;
                    
                    this.bin_ind(end,new_node_ind) = this.bin_ind(end, new_node_ind) + 1;
                    this.bin_ind(this.bin_ind(end,new_node_ind),new_node_ind) = cur_bin+this.bin_x;
                    if x_comp > x_left
                        this.bin(cur_bin-1+this.bin_x).last = this.bin(cur_bin-1+this.bin_x).last + 1;
                        this.bin(cur_bin-1+this.bin_x).nodes(this.bin(cur_bin-1+this.bin_x).last) = new_node_ind;
                        
                        this.bin_ind(end,new_node_ind) = this.bin_ind(end, new_node_ind) + 1;
                        this.bin_ind(this.bin_ind(end,new_node_ind),new_node_ind) = cur_bin-1+this.bin_x;
                    end
                    if x_comp < x_right && cur_bin+this.bin_x+1 <= this.nbins
                        this.bin(cur_bin+1+this.bin_x).last = this.bin(cur_bin+1+this.bin_x).last + 1;
                        this.bin(cur_bin+1+this.bin_x).nodes(this.bin(cur_bin+1+this.bin_x).last) = new_node_ind;
                        
                        this.bin_ind(end,new_node_ind) = this.bin_ind(end, new_node_ind) + 1;
                        this.bin_ind(this.bin_ind(end,new_node_ind),new_node_ind) = cur_bin+1+this.bin_x;
                    end
                end
            end
            
            if y_comp > y_bottom
                if cur_bin-this.bin_x > 0
                    this.bin(cur_bin-this.bin_x).last = this.bin(cur_bin-this.bin_x).last + 1;
                    this.bin(cur_bin-this.bin_x).nodes(this.bin(cur_bin-this.bin_x).last) = new_node_ind;
                    
                    this.bin_ind(end,new_node_ind) = this.bin_ind(end, new_node_ind) + 1;
                    this.bin_ind(this.bin_ind(end,new_node_ind),new_node_ind) = cur_bin-this.bin_x;
                    
                    if x_comp > x_left && cur_bin-1-this.bin_x > 0
                        this.bin(cur_bin-1-this.bin_x).last = this.bin(cur_bin-1-this.bin_x).last + 1;
                        this.bin(cur_bin-1-this.bin_x).nodes(this.bin(cur_bin-1-this.bin_x).last) = new_node_ind;
                        
                        this.bin_ind(end,new_node_ind) = this.bin_ind(end, new_node_ind) + 1;
                        this.bin_ind(this.bin_ind(end,new_node_ind),new_node_ind) = cur_bin-1-this.bin_x;
                    end
                    if x_comp < x_right
                        this.bin(cur_bin+1-this.bin_x).last = this.bin(cur_bin+1-this.bin_x).last + 1;
                        this.bin(cur_bin+1-this.bin_x).nodes(this.bin(cur_bin+1-this.bin_x).last) = new_node_ind;
                        
                        this.bin_ind(end,new_node_ind) = this.bin_ind(end, new_node_ind) + 1;
                        this.bin_ind(this.bin_ind(end,new_node_ind),new_node_ind) = cur_bin+1-this.bin_x;
                    end
                end
            end
        end
        
        %%% RRT* specific functions
        
        function neighbor_nodes = neighbors(this, new_node_position, nearest_node_ind)
            % seeks for neighbors and returns indices of neighboring nodes
            radius = this.delta_near;
            % % binning chunk of code
            x_comp = int32(new_node_position(1) / this.bin_size - 0.5);
            y_comp = int32(new_node_position(2) / this.bin_size - 0.5);
            
            cur_bin = x_comp + y_comp*this.bin_x + this.bin_offset;
            
            num_nbors = this.bin(cur_bin).last;
            
            if num_nbors < 20
                nbors = 1:this.nodes_added;
                num_nbors = this.nodes_added;
            else
                nbors = this.bin(cur_bin).nodes(1:num_nbors);
            end
            
            this.compare_table(1:num_nbors) = sum((this.tree(:, nbors) - repmat(new_node_position,1,num_nbors)).^2);
            % just in case
            if this.free_nodes_ind > 1
                this.compare_table(this.free_nodes(1:(this.free_nodes_ind-1))) = intmax;
            end
            [this.compare_table(1:num_nbors), this.index(1:num_nbors)] = sort(this.compare_table(1:num_nbors));
            temp = nbors(this.index((this.compare_table(1:num_nbors) <= radius^2*1.00001) & (this.compare_table(1:num_nbors) > 0 )));
            neighbor_nodes = temp;
            % neighbor_nodes = setdiff(temp, nearest_node_ind);
            
        end
        
        function min_node_ind = chooseParent(this, neighbors, nearest_node, new_node_position)
            % finds the node with minimal cummulative cost node from the root of
            % the tree. i.e. find the cheapest path end node.
            min_node_ind = nearest_node;
            min_cumcost = this.cumcost(nearest_node) + norm(this.tree(:, nearest_node)- new_node_position);
            for ind=1:numel(neighbors)
                temp_cumcost = this.cumcost(neighbors(ind)) + norm(this.tree(:, neighbors(ind)) - new_node_position);
                if temp_cumcost < min_cumcost && ~this.obstacle_collision(new_node_position, neighbors(ind))
                    min_cumcost = temp_cumcost;
                    min_node_ind = neighbors(ind);
                end
            end
        end
        
        function rewire(this, new_node_ind, neighbors, min_node_ind)
            % method looks thru all neighbors(except min_node_ind) and
            % seeks and reconnects neighbors to the new node if it is
            % cheaper
            queue = zeros(1, int32(this.max_nodes/5));
            for ind = 1:numel(neighbors)
                % omit
                if (min_node_ind == neighbors(ind))
                    continue;
                end
                temp_cost = this.cumcost(new_node_ind) + norm(this.tree(:, neighbors(ind)) - this.tree(:, new_node_ind));
                if (temp_cost < this.cumcost(neighbors(ind)) && ...
                        ~this.obstacle_collision(this.tree(:, new_node_ind), neighbors(ind)))
                    
                    %this.cumcost(neighbors(ind)) = temp_cost;
                    this.children(this.parent(neighbors(ind))) = this.children(this.parent(neighbors(ind))) - 1;
                    this.parent(neighbors(ind)) = new_node_ind;
                    this.children(new_node_ind) = this.children(new_node_ind) + 1;
                    this.num_rewired = this.num_rewired + 1;
                    
                    bottom = 0;
                    top = 0;
                    bottom = bottom + 1;
                    queue(bottom) = neighbors(ind);
                    delta_cost = temp_cost - this.cumcost(neighbors(ind));
                    
                    while top < bottom
                        top = top+1;
                        cur = queue(top);
                        this.cumcost(cur) = this.cumcost(cur)+delta_cost;
                        kids = this.list(this.parent == cur);
                        for k_ind = 1:numel(kids)
                            bottom = bottom + 1;
                            queue(bottom) = kids(k_ind);
                        end
                    end           
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
            this.tree(:, node_to_remove) = [intmax; intmax];
            this.free_nodes(this.free_nodes_ind) = node_to_remove;
            this.free_nodes_ind = this.free_nodes_ind + 1;
            
            for ind=1:this.bin_ind(end,node_to_remove)
                upd = setdiff(this.bin(this.bin_ind(ind, node_to_remove)).nodes(1:this.bin(this.bin_ind(ind, node_to_remove)).last), node_to_remove);
                this.bin(this.bin_ind(ind, node_to_remove)).nodes(1:numel(upd)) = upd;
            end
            this.bin_ind(end,node_to_remove) = 0;
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
            this.cost(reused_node_ind) = norm(this.tree(:, nearest_node) - new_node_position);
            this.cumcost(reused_node_ind) = this.cumcost(nearest_node) + this.cost(reused_node_ind);
            
            radius = this.delta_near;
            
            x_comp = int32(new_node_position(1) / this.bin_size - 0.5);
            y_comp = int32(new_node_position(2) / this.bin_size - 0.5);
            
            cur_bin = x_comp + y_comp*this.bin_x + this.bin_offset;
            
            this.bin(cur_bin).last = this.bin(cur_bin).last + 1;
            this.bin(cur_bin).nodes(this.bin(cur_bin).last) = reused_node_ind;
            
            this.bin_ind(end,reused_node_ind) = this.bin_ind(end, reused_node_ind) + 1;
            this.bin_ind(this.bin_ind(end, reused_node_ind), reused_node_ind) = cur_bin;
            
            %% placing nodes in additional bins
            x_left = x_comp;
            x_right = x_comp;
            y_top = y_comp;
            y_bottom = y_comp;
            if new_node_position(1) - radius >= this.XY_BOUNDARY(1)
                x_left = int32((new_node_position(1) - radius)/this.bin_size - 0.5);
            end
            if new_node_position(1) + radius <= this.XY_BOUNDARY(2)
                x_right = int32((new_node_position(1) + radius)/this.bin_size - 0.5);
            end
            if new_node_position(2) - radius >= this.XY_BOUNDARY(3)
                y_top = int32((new_node_position(2) + radius)/this.bin_size - 0.5);
            end
            if new_node_position(2) + radius <= this.XY_BOUNDARY(4)
                y_bottom = int32((new_node_position(2) - radius)/this.bin_size - 0.5);
            end
            
            if x_comp > x_left && cur_bin - 1 > 0
                this.bin(cur_bin-1).last = this.bin(cur_bin-1).last + 1;
                this.bin(cur_bin-1).nodes(this.bin(cur_bin-1).last) = reused_node_ind;
                
                this.bin_ind(end,reused_node_ind) = this.bin_ind(end, reused_node_ind) + 1;
                this.bin_ind(this.bin_ind(end,reused_node_ind),reused_node_ind) = cur_bin-1;
            end
            
            if x_comp < x_right && cur_bin + 1 < this.nbins
                this.bin(cur_bin+1).last = this.bin(cur_bin+1).last + 1;
                this.bin(cur_bin+1).nodes(this.bin(cur_bin+1).last) = reused_node_ind;
                
                this.bin_ind(end,reused_node_ind) = this.bin_ind(end, reused_node_ind) + 1;
                this.bin_ind(this.bin_ind(end, reused_node_ind), reused_node_ind) = cur_bin+1;
            end
            
            if y_comp < y_top
                if cur_bin+this.bin_x <= this.nbins
                    this.bin(cur_bin+this.bin_x).last = this.bin(cur_bin+this.bin_x).last + 1;
                    this.bin(cur_bin+this.bin_x).nodes(this.bin(cur_bin+this.bin_x).last) = reused_node_ind;
                    
                    this.bin_ind(end,reused_node_ind) = this.bin_ind(end, reused_node_ind) + 1;
                    this.bin_ind(this.bin_ind(end,reused_node_ind),reused_node_ind) = cur_bin+this.bin_x;
                    if x_comp > x_left
                        this.bin(cur_bin-1+this.bin_x).last = this.bin(cur_bin-1+this.bin_x).last + 1;
                        this.bin(cur_bin-1+this.bin_x).nodes(this.bin(cur_bin-1+this.bin_x).last) = reused_node_ind;
                        
                        this.bin_ind(end,reused_node_ind) = this.bin_ind(end, reused_node_ind) + 1;
                        this.bin_ind(this.bin_ind(end,reused_node_ind),reused_node_ind) = cur_bin-1+this.bin_x;
                    end
                    if x_comp < x_right && cur_bin+this.bin_x+1 <= this.nbins
                        this.bin(cur_bin+1+this.bin_x).last = this.bin(cur_bin+1+this.bin_x).last + 1;
                        this.bin(cur_bin+1+this.bin_x).nodes(this.bin(cur_bin+1+this.bin_x).last) = reused_node_ind;
                        
                        this.bin_ind(end,reused_node_ind) = this.bin_ind(end, reused_node_ind) + 1;
                        this.bin_ind(this.bin_ind(end,reused_node_ind),reused_node_ind) = cur_bin+1+this.bin_x;
                    end
                end
            end
            
            if y_comp > y_bottom
                if cur_bin-this.bin_x > 0
                    this.bin(cur_bin-this.bin_x).last = this.bin(cur_bin-this.bin_x).last + 1;
                    this.bin(cur_bin-this.bin_x).nodes(this.bin(cur_bin-this.bin_x).last) = reused_node_ind;
                    
                    this.bin_ind(end,reused_node_ind) = this.bin_ind(end, reused_node_ind) + 1;
                    this.bin_ind(this.bin_ind(end,reused_node_ind),reused_node_ind) = cur_bin-this.bin_x;
                    
                    if x_comp > x_left && cur_bin-1-this.bin_x > 0
                        this.bin(cur_bin-1-this.bin_x).last = this.bin(cur_bin-1-this.bin_x).last + 1;
                        this.bin(cur_bin-1-this.bin_x).nodes(this.bin(cur_bin-1-this.bin_x).last) = reused_node_ind;
                        
                        this.bin_ind(end,reused_node_ind) = this.bin_ind(end, reused_node_ind) + 1;
                        this.bin_ind(this.bin_ind(end,reused_node_ind),reused_node_ind) = cur_bin-1-this.bin_x;
                    end
                    if x_comp < x_right
                        this.bin(cur_bin+1-this.bin_x).last = this.bin(cur_bin+1-this.bin_x).last + 1;
                        this.bin(cur_bin+1-this.bin_x).nodes(this.bin(cur_bin+1-this.bin_x).last) = reused_node_ind;
                        
                        this.bin_ind(end,reused_node_ind) = this.bin_ind(end, reused_node_ind) + 1;
                        this.bin_ind(this.bin_ind(end,reused_node_ind), reused_node_ind) = cur_bin+1-this.bin_x;
                    end
                end
            end
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
                this.plot_circle(this.obstacle.cir_center{k}(1),this.obstacle.cir_center{k}(2), this.obstacle.r(k));
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
        function plot_circle(x, y, r)
            t = 0:0.001:2*pi;
            cir_x = r*cos(t) + x;
            cir_y = r*sin(t) + y;
            plot(cir_x, cir_y, 'r-', 'LineWidth', 1.5);
        end
    end
end
