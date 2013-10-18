classdef FNRedundantManipulator < handle
    properties (SetAccess = private)
        tree                    % Array stores states 
        parent                  % Array stores relation information about nodes                    
        children                % Number of children
        free_nodes              % Indices of free nodes
        free_nodes_ind          % Last element in free_nodes
        cost                    % Cost between 2 connected states
        cumcost                 % Cost from the root of the tree to the given node
        conf                    % configuration struct
        XY_BOUNDARY             %[min_x max_x min_y max_y]
        start_point             
        goal_point 
        delta_goal_point        % radius of goal position
        delta_ang_max           % maximal change in angle for each of the joints while adding new node
        delta_ang_neighbor      % maximal change in angle for each of the joints while seeking for neighbors
        nodes_added             % the length of tree
        max_ang                 % maximum angle a joint can go
        min_ang                 % minimum angle a joint can go
        num_link                % number of links of Redudant Manipulator (DOF)
        len_link                % length of each of link
        width_link              % width of each of link
        height_link             % height of each link
        obstacle                % obstacle information
        
        best_path_node 
        goal_reached 
        
        %%% temporary variables
        
        compare_table 
        index
        list
        temp_new_node_position
        num_rewired
        
        %%% obstacle detection specific
        turn_states             % intermediate angle states from one configuration of redudant robot manipulator to another
        turn_pos                % intermediate position states from one configuration of redudant robot manipulator to another
        turn_cumsum             % intermediate states, cummulative sum
        test_list               % combination of links for self collision check up
        num_comb                % number of combinations
        link_cen                % positions of centers of links
        link_corner             % OBB(Oriented Bounding Boxes) of each links
        link_axis               % 
        link_quant              % Link quantization
        obs_mat                 % special matrix for obstacle collision
        step_div                % number of intermediate step between two states
        temp_obs                
    end
    methods
        % object constructor
        function this = FNRedundantManipulator(rand_seed, max_nodes, map, conf)
            rng(rand_seed);
            this.conf = conf;
            this.tree = struct;
            this.tree.angle = zeros(conf.num_link, max_nodes, 'single');
            this.tree.position = zeros(conf.num_link, 2, max_nodes, 'single');
            this.temp_new_node_position = zeros(conf.num_link, 2, 'single');
                
            this.parent = zeros(1, max_nodes, 'int32');
            this.children = zeros(1, max_nodes, 'int32');
            this.free_nodes = zeros(1, max_nodes, 'int32');
            this.free_nodes_ind = int32(1);
            
            this.cost = zeros(1, max_nodes, 'single');
            this.cumcost = zeros(1, max_nodes, 'single');
            this.XY_BOUNDARY = zeros(4,1, 'single');
            this.tree.angle(:, 1) = conf.init_conf;
            this.start_point = map.start_point;
            this.goal_point = map.goal_point;
            this.delta_goal_point = conf.delta_goal_point;
            this.delta_ang_max = conf.delta_ang_max;
            this.delta_ang_neighbor = conf.delta_ang_neighbor;
            this.nodes_added = 2;
            this.min_ang = conf.min_ang;
            this.max_ang = conf.max_ang;
            this.len_link = conf.len_link;
            this.num_link = conf.num_link;
            this.width_link = conf.width_link;
            this.height_link = conf.height_link;
            
            %%% obstacle detection specific
            this.step_div = conf.step_div;
            this.turn_states = zeros(this.num_link, this.step_div, 'single'); % angles
            this.turn_pos = zeros(this.num_link, 2, this.step_div, 'single'); % positions
            this.turn_cumsum = zeros(this.num_link, this.step_div, 'single'); % cum sum
            
            this.test_list = combnk(1:this.num_link,2);                               % 2uple combinations of 5 links
            this.test_list = setdiff(this.test_list,  [ (1:this.num_link-1)' (2:this.num_link)'], 'rows');    % Remove adjacent links
            [this.num_comb, ~] = size(this.test_list);
            
            this.link_corner = cell(this.num_link, 1);
            this.link_axis = cell(this.num_link, 2);
            this.link_quant = 10;
            this.load_map(map.name);
            this.update_link_position(1);
            
            
            %%% temp var-s initialization
            this.temp_obs = zeros(this.num_link+1, 2, 'single');
            this.compare_table = zeros(1, max_nodes, 'single');
            this.index = zeros(1, max_nodes, 'single');
            this.num_rewired = 0;
            
            this.list = 1:max_nodes;
        end

        function state = sample(this)
            % generate random angle within min_ang and max_ang range
            state = (this.max_ang - this.min_ang) .* rand(this.num_link, 1) + this.min_ang;
            
            % generate manipulator orientation
            link_pos = this.generate_link_position(state);
            % check if manipulator within the defined area
            while any(link_pos(:,1) < this.XY_BOUNDARY(1)) || any(link_pos(:,1) > this.XY_BOUNDARY(2)) || any(link_pos(:,2) < this.XY_BOUNDARY(3)) || any(link_pos(:,2) > this.XY_BOUNDARY(4))
                state = (this.max_ang - this.min_ang) .* rand(this.num_link, 1) + this.min_ang;
                link_pos = this.generate_link_position(state);
            end
        end

        function node_index = nearest(this, new_node)
            %%% 1: nearest by joint positions
            node_index = this.next_node_by_len(new_node);
            %%% 2: nearest by angle distance
            %node_index = this.next_node_by_ang(temp_ang);
        end

        function [ nearest_node ] = next_node_by_len(this, temp_ang)
            %NEXT_NODE_BY_LEN Finds the nearest node by end effector
            % position. Euclidian distance is used.
           
            link_pos_temp = zeros(size(temp_ang,1),2);
            temp_world_ang = cumsum( temp_ang );
            
            link_pos_temp(:, 1) = cumsum( this.len_link.*cos(temp_world_ang));
            link_pos_temp(:, 2) = cumsum( this.len_link.*sin(temp_world_ang));
            
            % find index of minimumal distance node 
            [~, minFromPointInd] = min(...
                sum((squeeze(this.tree.position(:, 1, 1:this.nodes_added-1)) - repmat(link_pos_temp(:,1), 1, this.nodes_added-1)) .^ 2) ...
                + sum((squeeze(this.tree.position(:, 2, 1:this.nodes_added-1)) - repmat(link_pos_temp(:,2), 1, this.nodes_added-1)) .^ 2) ...
                );
            nearest_node = minFromPointInd;
        end
        
        function position = steer(this, nearest_node_ind, new_node)
            % in this function we try to go in the direction of a given
            % new_node node if it is to far and we cannot reach it in one
            % step. 
            
            from = this.tree.angle(:, nearest_node_ind);
            to = new_node;
            angle_diff = to - from;
            angle_inds = abs(angle_diff) > this.delta_ang_max;
            
            node = zeros(this.num_link, 1);
            
            % Making the state nice if it was far away from the nearest state
            
            node(angle_inds) = from(angle_inds) + sign(angle_diff(angle_inds)) * this.delta_ang_max;
            node(~angle_inds) = to(~angle_inds);
            
            % return the generated state
            position = node;
        end

        function load_map(this, map_name)
            % function loads '.mat' file with obstacle information and the
            % size of the map
            
            if strcmp(map_name ,'null.mat')
                this.obstacle.output = cell(1,1);
                this.obstacle.num = 0;
                this.XY_BOUNDARY = [-45 45 0 90];
            else
                map_path = 'maps/';
                this.obstacle = load([map_path map_name], 'num', 'output', 'x_constraints', 'y_constraints');
                this.XY_BOUNDARY = [this.obstacle.x_constraints this.obstacle.y_constraints];
                
                %%% Circular obstacles init
                
                this.obs_mat = cell(this.obstacle.num, 1);
                for ind2 = 1:this.obstacle.num
                    % generate matrix so that we can vectorize collision
                    % detection code with circular obstacles
                    this.obs_mat{ind2} = repmat(this.obstacle.output{ind2}(1:2) , this.link_quant*(this.num_link-1), 1);
                end
            end
        end

        function collision = obstacle_collision(this, new_node, node_index)
            
            % generate this.step_div number of states to prevent omission
            for link_ind = 1:this.num_link
                this.turn_states(link_ind, :) = linspace(this.tree.angle(link_ind, node_index), new_node(link_ind), this.step_div);
            end
            
            % generate positions for all the states we created
            for ind1 = 1:this.step_div
                
                this.turn_cumsum(:, ind1) = cumsum(this.turn_states(:, ind1));
                this.turn_pos(:,1,ind1) = cumsum(this.len_link .* cos(this.turn_cumsum(:, ind1)))+this.start_point(1);
                this.turn_pos(:,2,ind1) = cumsum(this.len_link .* sin(this.turn_cumsum(:, ind1)))+this.start_point(2);
                
                this.temp_obs(1:this.num_link+1, :) = [this.start_point(1) this.start_point(2); this.turn_pos(:, :, ind1)];
                this.link_cen(1:this.num_link, :) = (this.temp_obs(1:this.num_link,:) + this.turn_pos(:, :, ind1))/2;      % Center of the links
                
                % generate corner position of obb of manipulator
                for ind2 = 1:this.num_link
                    x1 = this.height_link(ind2)/2*[cos(this.turn_cumsum(ind2, ind1)) sin(this.turn_cumsum(ind2, ind1)) ]+this.start_point(1);
                    y1 = this.width_link(ind2)/2*[-sin(this.turn_cumsum(ind2, ind1)) cos(this.turn_cumsum(ind2, ind1)) ]+this.start_point(2);
                    temp = this.link_cen(ind2,:);
                    this.link_corner{ind1,ind2} = temp(ones(4,1),:) + [ -x1-y1; x1-y1; x1+y1; -x1+y1];     % Corners of the OBBs
                    this.link_axis{ind2,1} = (this.link_corner{ind1,ind2}(2,:) - this.link_corner{ind1,ind2}(1,:)) ;
                    this.link_axis{ind2,2} = (this.link_corner{ind1,ind2}(4,:) - this.link_corner{ind1,ind2}(3,:)) ;
                end
                
                % check for self collision, excluding consecutive links
                for ind3 = 1:this.num_comb
                    ind_links = this.test_list(ind3,:);
                    collision = detect_obb_collision(this.link_cen(ind_links, :), this.link_corner(ind1, ind_links), this.link_axis(ind_links, :));
                    if collision
                        return;
                    end
                end
            end
            % check for collision of circular obstacles
            collision = this.link_obstacle_collision();
        end
        
        function collision = link_obstacle_collision(this)
            % method checks wheather any of links collide obstacles
            
            collision = false;
            
            for ind1 = 1:this.num_link
                
                A = [this.link_corner{:, ind1}]';
                x = double(A(1:2:end)');
                y = double(A(2:2:end)');
                % generate convex hull
                k = convhull(x,y);
                maxx = 0;
                maxy = 0;
                maxxpair =[0 0];
                maxypair =[0 0];
                % find maximum and minimum point on x and y axises.
                for ind2 = 1:numel(k)
                    for ind3 = ind2+1:numel(k)
                        if abs(x(k(ind2)) - x(k(ind3))) > maxx
                            maxx = abs(x(k(ind2)) - x(k(ind3)));
                            maxxpair(1:2) = [k(ind2) k(ind3)];
                        end
                        if abs(y(k(ind2)) - y(k(ind3))) > maxy
                            maxy = abs(y(k(ind2)) - y(k(ind3)));
                            maxypair(1:2) = [k(ind2) k(ind3)];
                        end
                    end
                end
                % compute center of AABB(Axis aligned bounding boxes)
                c = [mean(x(maxxpair)) mean(y(maxypair))];
                % compute radii in x and y direction
                r_x = maxx/2;
                r_y = maxy/2;
                
                % check thru all circular obstacle
                for ind2 = 1 : this.obstacle.num
                    % just check if distance between centers of AABB and
                    % circular obstacle less than sum of radii of
                    % AABB(foreach of axises) and circluar obstacle
                    if (abs(c(1)-this.obstacle.output{ind2}(1)) <= (r_x + this.obstacle.output{ind2}(3)))...
                            && (abs(c(2)-this.obstacle.output{ind2}(2)) <= (r_y + this.obstacle.output{ind2}(3)))
                        for ind3 = 1:this.step_div
                            % addition more expensive collision detection
                            collision = this.link_obstacle_collision_conservative(ind3);
                            if collision
                                return
                            end
                        end
                    end
                end
            end
        end
        
        function collision = link_obstacle_collision_conservative(this, turn_ind)
            % methods uses pregenerated states in obstacle_collision method
            % divides line respresentation of link in this.link_quant
            % pieces and check for collision with a circular obstacle
            collision = false;
            
            rob_fin_x = zeros(this.link_quant*(this.num_link-1), 1);
            rob_fin_y = zeros(this.link_quant*(this.num_link-1), 1);
            
            
            for ind1 = 1:this.num_link-1
                temp_ind = this.link_quant*(ind1-1)+1: this.link_quant*ind1;
                rob_fin_x(temp_ind) = linspace(this.turn_pos(ind1,1,turn_ind), this.turn_pos(ind1+1,1,turn_ind), this.link_quant);
                rob_fin_y(temp_ind) = linspace(this.turn_pos(ind1,2,turn_ind), this.turn_pos(ind1+1,2,turn_ind), this.link_quant);
            end
            
            for ind2 = 1 : this.obstacle.num
                dist1 =  sum( ([rob_fin_x, rob_fin_y] - this.obs_mat{ind2} ).^2 , 2);
                if any(dist1 <= (this.obstacle.output{ind2}(3) + (this.width_link(1)/2))^2 )
                    collision = true;
                    return;
                end
            end
        end
        
        function new_node_ind = insert_node(this, parent_node_ind, new_node)
            % method insert new node in the tree
            new_node_ind = this.nodes_added;
            this.nodes_added = this.nodes_added + 1;
            this.tree.angle(:, new_node_ind) = new_node;
            this.update_link_position(new_node_ind);
            this.parent(new_node_ind) = parent_node_ind;
            this.children(parent_node_ind) = this.children(parent_node_ind) + 1; 
            this.update_link_position(new_node_ind);
            this.cost(new_node_ind) = this.calc_cost_euclidian(parent_node_ind, this.tree.position(:, :, new_node_ind));
            this.cumcost(new_node_ind) = this.cumcost(parent_node_ind) + this.cost(new_node_ind);
        end
        
        %%% RRT* specific functions
        
        function neighbor_nodes = neighbors(this, new_node, nearest_node_ind)
            % why do we seek nearest using link positions, however
            % neighbors using angles?
            % clarify this
            temp = abs(this.tree.angle(:, 1:this.nodes_added-1) - repmat(new_node, 1, this.nodes_added-1)) > this.delta_ang_neighbor;
            %temp(:, nearest_node_ind) = this.tree.angle(:, 1) * 0;
            ind = 1:this.nodes_added-1;
            temp = ind(~sum(temp));
            neighbor_nodes = setdiff(temp, nearest_node_ind);
        end
        
        function min_node_ind = chooseParent(this, neighbors, nearest_node_ind, new_node)
            % finds the node with minimal cummulative cost node from the root of
            % the tree. i.e. find the cheapest path end node.
            min_node_ind = nearest_node_ind;
            this.temp_new_node_position = this.generate_link_position(new_node);
            min_cumcost = this.cumcost(nearest_node_ind) + this.calc_cost_euclidian(nearest_node_ind, this.temp_new_node_position);
            for ind=1:numel(neighbors)
                if(~this.obstacle_collision(new_node, neighbors(ind)))
                    temp_cumcost = this.cumcost(neighbors(ind)) + this.calc_cost_euclidian(neighbors(ind), this.temp_new_node_position);
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
                % omit node with 
                if (min_node_ind == neighbors(ind))
                    continue;
                end
                temp_cost = this.cumcost(new_node_ind) + this.calc_cost_euclidian(neighbors(ind), this.tree.position(:, :, new_node_ind));
                if (temp_cost < this.cumcost(neighbors(ind)) && ...
                        ~this.obstacle_collision(this.tree.position(:, new_node_ind), neighbors(ind)))
                    this.cumcost(neighbors(ind)) = temp_cost;
                    this.children(this.parent(neighbors(ind))) = this.children(this.parent(neighbors(ind))) - 1;
                    this.parent(neighbors(ind)) = new_node_ind;
                    this.children(new_node_ind) = this.children(new_node_ind) + 1;
                    this.num_rewired = this.num_rewired + 1;
                end
            end
        end
        
        %%%%%%%%%%%%%%%%%%%%%%%%%
        function plot(this)
            radius = 2;
            % obtain path
            [backtrace_path, path_iter] = this.evaluate_path();
            % smoothen the trajectory. manipulator will move without jerks
            [ xx, yy, trajectory] = this.smooth_trajectory(backtrace_path, path_iter);
            fig1 = figure;
            % make plot window bigger
            set(fig1, 'Position', [50 50 1600 800]);
            
            % two subplot is on the plot
            % left subplot will draw the manipulator's motion
            % right one will draw the tree of the position of end effector
            
            subplot(1,2,1);
            % preparing to draw circle which will indicate the destination
            hold on;
            p1 = this.plot_circle(this.goal_point(1), this.goal_point(2), this.delta_goal_point);
            set(p1, 'HandleVisibility','off');
            
            % obstacle drawing
            this.draw_obstacles();
            
            grid on; axis square; axis(this.XY_BOUNDARY);
            title('Flexible Robot Manipulator','FontWeight','Demi');
            xlabel(' Position (m) ' , 'FontWeight', 'Demi');
            ylabel(' Position (m) ' , 'FontWeight', 'Demi');
            hold off;
            
            % right plot
            subplot(1,2,2);
            hold on;
            
            % plots the tree of end effector position
            drawn_nodes = zeros(1, this.nodes_added-1);
            for ind = this.nodes_added-1:-1:1;
                current_index = ind;
                while(current_index ~= 1) && this.parent(current_index) ~= -1
                    % avoid drawing same nodes twice or more times
                    if(drawn_nodes(current_index) == false || drawn_nodes(this.parent(current_index)) == false)
                        plot([this.tree.position(end, 1, current_index);this.tree.position(end, 1, this.parent(current_index))], ...
                            [this.tree.position(end, 2, current_index);this.tree.position(end, 2, this.parent(current_index))],'g-','LineWidth', 0.1);
                        plot([this.tree.position(end, 1, current_index);this.tree.position(end, 1, this.parent(current_index))], ...
                            [this.tree.position(end, 2, current_index);this.tree.position(end, 2, this.parent(current_index))],'.k');
                        drawn_nodes(current_index) = true;
                    end
                    current_index = this.parent(current_index);
                end
            end
            
            p1 = plot(squeeze(trajectory(:, 1, :))', squeeze(trajectory(:,2,:))' , 'LineWidth', 3);
            %
            
            % obstacle drawing
            this.draw_obstacles();
            
            grid on; axis square; axis(this.XY_BOUNDARY);
            title('Rapidly Exploring Random Tree','FontWeight','Demi');
            xlabel(' Position (m) ' , 'FontWeight', 'Demi');
            ylabel(' Position (m) ' , 'FontWeight', 'Demi');
            hold off;
            
            
            %% remaining code draws the motion of the robot
            % line to fix the video output problem
            set(gcf,'renderer','opengl');
            
            smooth_ang = zeros(this.num_link, size(xx, 2));
            smooth_trajectory = zeros(this.num_link, 2, size(xx, 2));
            for ind = 1:size(xx,2)
                smooth_ang(:,ind) = cumsum( yy(:,ind) );
                smooth_trajectory(:, 1, ind) = cumsum( this.len_link.*cos(smooth_ang(:,ind) )) + this.start_point(1);
                smooth_trajectory(:, 2, ind) = cumsum( this.len_link.*sin(smooth_ang(:,ind) )) + this.start_point(2);
                
                % plot on the left subplot
                subplot(1,2,1);
                cla;
                hold on;
                
                % initial joint of the redundant robot manipulator
                this.plot_circle(this.start_point(1), this.start_point(2), radius);
                for ind2=1:this.num_link
                    
                    fill([ smooth_trajectory(ind2, 1, ind) - radius * sin(smooth_ang(ind2,ind)); ...
                        smooth_trajectory(ind2, 1, ind) - radius * sin(smooth_ang(ind2,ind)) - this.len_link(ind2) * cos(smooth_ang(ind2,ind));       ...
                        smooth_trajectory(ind2, 1, ind) + radius * sin(smooth_ang(ind2,ind)) - this.len_link(ind2) * cos(smooth_ang(ind2,ind)); ...
                        smooth_trajectory(ind2, 1, ind) + radius * sin(smooth_ang(ind2,ind))], ...
                        [ smooth_trajectory(ind2, 2, ind) + radius * cos(smooth_ang(ind2,ind)); ...
                        smooth_trajectory(ind2, 2, ind) + radius * cos(smooth_ang(ind2,ind)) - this.len_link(ind2) * sin(smooth_ang(ind2,ind)) ; ...
                        smooth_trajectory(ind2, 2, ind) - radius * cos(smooth_ang(ind2,ind)) - this.len_link(ind2) * sin(smooth_ang(ind2,ind)); ...
                        smooth_trajectory(ind2, 2, ind) - radius * cos(smooth_ang(ind2,ind))] ...
                        ,'g');
                    
                    s3 = this.plot_circle(smooth_trajectory(ind2, 1, ind), smooth_trajectory(ind2, 2, ind), radius);
                end
                drawnow; % draw the manipulator immediately 
                hold off;
            end
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
        
        function update_link_position(this, ind)
            % Update info about the manipulator
            world_ang = cumsum( this.tree.angle(:, ind) );
            % Compute the link end positions
            this.tree.position(:, 1, ind) = cumsum( this.len_link.*cos(world_ang )) + this.start_point(1);
            this.tree.position(:, 2, ind) = cumsum( this.len_link.*sin(world_ang )) + this.start_point(1);
        end
        
        function link_pos = generate_link_position(this, node)
            % generates position and orientation of the redundant
            % manipulator model
            link_pos = zeros(this.num_link, 2);
            temp_world_ang = cumsum( node );
            % Compute the link end positions
            link_pos(:, 1) = cumsum( this.len_link.*cos(temp_world_ang)) + this.start_point(1);
            link_pos(:, 2) = cumsum( this.len_link.*sin(temp_world_ang)) + this.start_point(1);
        end
        
        function cost = calc_cost_euclidian(this, from_ind, dest_node)
            % square root is not taken
            cost = norm(this.tree.position(:, :, from_ind) - dest_node, 'fro');
        end
        
        function cost = calc_cost_angle(this, from_ind, dest_node)
            % square root is not taken
            cost = norm(this.tree.angle(:, from_ind) - dest_node, 'fro');
        end
        
        
        function [backtrace_path, path_iter] = evaluate_path(this)
            % method find the cheapest path and the number if nodes in this
            % path
            distances = zeros(this.nodes_added-1, 2);
            distances(:, 1) = (this.tree.position(end, 1, 1:this.nodes_added-1) - this.goal_point(1)) .^ 2 ...
                + (this.tree.position(end, 2, 1:this.nodes_added-1) -  this.goal_point(2)) .^ 2;
            distances(:, 2) = 1:this.nodes_added-1;
            distances = sortrows(distances, 1);
            distances(:, 1) = distances(:, 1) <= (this.delta_goal_point ^ 2);
            dist_index = numel(find(distances(:, 1) == 1));
            
            % find the cheapest path
            if(dist_index ~= 0)
                min_cost = inf;
                for ind=1:dist_index
                    if(distances(ind,1) == 1)
                        temp_cost = this.cumcost(distances(ind,2));
                        if(min_cost > temp_cost)
                            min_cost = temp_cost;
                            nearest_node_index = distances(ind,2);
                        end
                    end
                end
            else
                disp('NOTICE! Robot cannot reach the goal');
                nearest_node_index = distances(1,2);
            end
            
            % backtracing the path
            current_index = nearest_node_index;
            path_iter = 1;
            backtrace_path = zeros(1,1);
            while(current_index ~= 1 )
                backtrace_path(path_iter) = current_index;
                path_iter = path_iter + 1;
                current_index = this.parent(current_index);
                if path_iter > 100
                    break;
                end
            end
            backtrace_path(path_iter) = current_index;
        end
        
        function [ xx, yy, smooth_trajectory ] = smooth_trajectory(this, backtrace_path, path_iter )
            %SMOOTH_TRAJECTORY Smooth the initial trajectory to avoid jerked motion
            
            joint_pos_states = zeros(this.num_link, 2, path_iter);
            joint_ang_states = zeros(this.num_link, path_iter);
            
            for ind = path_iter:-1:1
                joint_pos_states(:,:, ind) = this.tree.position(:,:, backtrace_path(ind));
                joint_ang_states(:, ind) = this.tree.angle(:, backtrace_path(ind));
            end
            
            xx = 1:0.1:path_iter;
            x = 1:1:path_iter;
            yy = zeros(this.num_link, size(xx,2));
            for temp_ind = 1:this.num_link
                yy(temp_ind, :) = cubic_spline(x, squeeze(joint_ang_states(temp_ind, path_iter:-1:1)), xx);
            end
            
            smooth_ang = zeros(this.num_link, size(xx,2));
            smooth_trajectory = zeros(this.num_link, 2, size(xx,2));
            for ind = 1:size(xx,2)
                smooth_ang(:,ind) = cumsum( yy(:,ind) );
                smooth_trajectory(:, 1, ind) = cumsum( this.len_link.*cos(smooth_ang(:,ind) ));
                smooth_trajectory(:, 2, ind) = cumsum( this.len_link.*sin(smooth_ang(:,ind) ));
            end
            
        end
        
        function draw_obstacles(this)
            % method draws all obstacles
            for k = 1:this.obstacle.num
                s2p2 = this.plot_disk(this.obstacle.output{k}(1), this.obstacle.output{k}(2), this.obstacle.output{k}(3));
                set(s2p2,'HandleVisibility','off','EdgeAlpha',0);
            end
        end
        
        %%% RRT*FN specific functions
        
        function best_path_evaluate(this)
            % computes the best path and index of the last node of the best
            % path
            distances = zeros(this.nodes_added-1, 2);
            distances(:, 1) = (this.tree.position(end, 1, 1:this.nodes_added-1) - this.goal_point(1)) .^ 2 ...
                + (this.tree.position(end, 2, 1:this.nodes_added-1) -  this.goal_point(2)) .^ 2;
            distances(:, 2) = 1:this.nodes_added-1;
            distances = sortrows(distances, 1);
            distances(:, 1) = distances(:, 1) <= (this.delta_goal_point ^ 2);
            dist_index = numel(find(distances(:, 1) == 1));
            
            % find the cheapest path
            if(dist_index ~= 0)
                min_cost = inf;
                for ind=1:dist_index
                    if(distances(ind,1) == 1)
                        temp_cost = this.cumcost(distances(ind,2));
                        if(min_cost > temp_cost)
                            min_cost = temp_cost;
                            nearest_node_index = distances(ind,2);
                        end
                    end
                end
            else
                disp('NOTICE! Robot cannot reach the goal');
                nearest_node_index = distances(1,2);
            end
            
            % backtracing the path
            this.best_path_node = nearest_node_index;
        end
        
        function forced_removal(this)
            % method removes any random point which does not have any
            % children, !!except!! the proposed best path nodes
            candidate = this.list(this.children(1:(this.nodes_added-1)) == 0);
            node_to_remove = candidate(randi(numel(candidate)));
            while node_to_remove == this.best_path_node
                node_to_remove = candidate(randi(numel(candidate)));
            end
            this.children(this.parent(node_to_remove)) = this.children(this.parent(node_to_remove)) - 1;
            this.parent(node_to_remove) = -1;
            this.tree.position(:, :, node_to_remove) = intmax;
            this.tree.angle(:, node_to_remove) = intmax;
            this.free_nodes(this.free_nodes_ind) = node_to_remove;
            this.free_nodes_ind = this.free_nodes_ind + 1;
        end
        
        function reused_node_ind = reuse_node(this, parent_node_ind, new_node)
            % method inserts new node instead of the removed one.
            if(this.free_nodes_ind == 1)
                disp('ERROR: Cannot find any free node!!!');
                return;
            end
            this.free_nodes_ind = this.free_nodes_ind - 1;
            reused_node_ind = this.free_nodes(this.free_nodes_ind);
            this.tree.angle(:, reused_node_ind) = new_node;
            this.update_link_position(reused_node_ind);
            this.parent(reused_node_ind) = parent_node_ind;
            this.children(parent_node_ind) = this.children(parent_node_ind) + 1; 
            this.cost(reused_node_ind) = this.calc_cost_euclidian(parent_node_ind, this.tree.position(:, :, reused_node_ind));
            this.cumcost(reused_node_ind) = this.cumcost(parent_node_ind) + this.cost(reused_node_ind);
        end
    end
    
    methods(Static)
        function hCirc = plot_circle(x, y, r)
            t = 0:0.001:2*pi;
            cir_x = r*cos(t) + x;
            cir_y = r*sin(t) + y;
            hCirc = plot(cir_x, cir_y, 'b-', 'LineWidth', 1.5);
        end
        
        function hDisk = plot_disk(x, y, r)
            t = 0:0.001:2*pi;
            cir_x = r*cos(t) + x;
            cir_y = r*sin(t) + y;
            hDisk = fill(cir_x, cir_y, 'r');
        end
        
        function delay(sec)
            tic;
            while toc < sec
            end
        end
    end
end