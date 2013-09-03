function update_obstacles(this)

% dumb initialization, @TODO better
if empty(this.dynamic_obstacle)
    steps = 20;
    step_len = 0.1;
    
    this.dymanic_obstacle.num = 1;
    this.dynamic_obstacle.r = cell(1, this.dymanic_obstacle.num);
    this.dynamic_obstacle.pos = cell(1, this.dymanic_obstacle.num);
    this.dynamic_obstacle.steps = cell(1, this.dymanic_obstacle.num);
    this.dynamic_obstacle.step_len = cell(1, this.dymanic_obstacle.num);
    
    this.dynamic_obstacle.r{1} = 0.8;
    this.dynamic_obstacle.pos{1} = [-10 10];
    this.dynamic_obstacle.steps{1} = steps;
    this.dynamic_obstacle.step_len{1} = step_len;
end

% movement control
for ind = 1:this.dymanic_obstacle.num    
    if this.dynamic_obstacle.steps{ind} == 0
        this.dynamic_obstacle.direction{ind} = int32(rand()*360);
        this.dynamic_obstacle.steps{ind} = int32(rand()*50);
        this.dynamic_obstacle.step_len{ind} = (1-0.1)*rand()+0.1;
    end
end

disp('Obstacle updated');
end