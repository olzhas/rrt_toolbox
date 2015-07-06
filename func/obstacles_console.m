%% 03/16/2015

close all;
clear all;
disp(' ');
disp('########################################################');
disp('Please keep in mind only CONVEX obstacles are supported.');
disp('########################################################');
disp(' ');
num = input('Enter number of obstacles: ');

x_constraints = zeros(1,2);
y_constraints = zeros(1,2);
disp('Enter contraints');

x_constraints(1,1) = input('... x_min: ');
x_constraints(1,2) = input('... x_max: ');
if (x_constraints(1) > x_constraints(2))
    disp('Error x_min > x_max');
    return;
end

y_constraints(1,1) = input('... y_min: ');
y_constraints(1,2) = input('... y_max: ');
if (y_constraints(1) > y_constraints(2))
    disp('Error y_min > y_max');
    return;
end

output = cell(1,num);
%%
for ind = 1: num;
    vert = input(['Obstacle #' num2str(ind) ': Enter number of vertices [min=3] -> ' ]);
    
    while vert <= 2
        vert = input('Number of vertices must be greater than 2... ');
    end
    points = zeros(vert, 2);
    for ind_vert = 1:vert
        display(['Vertex #' num2str(ind_vert) ':']);
        points(ind_vert,1) = input('Enter x: ');
        while(points(ind_vert,1) < x_constraints(1) || points(ind_vert,1) > x_constraints(2))
            points(ind_vert,1) = input('Error x is not in range, enter x: ');
        end
        points(ind_vert,2) = input('Enter y: ');
        while(points(ind_vert,2) < y_constraints(1) || points(ind_vert,2) > y_constraints(2))
            points(ind_vert,2) = input('Error y is not in range, enter y: ');
        end
    end
    points(vert+1,:) = points(1,:);
    output(ind) = {points(1:vert+1, :)};
end
%%
showMap = input('Do you want to look at your map? (y/n) ', 's');
if (strcmp(showMap, 'y') || strshowMap == 'Y')
    figure();
    hold on;
    for k = 1:num
        fill(output{k}(1:end, 1), output{k}(1:end, 2), 'r');
    end
    axis([x_constraints(1,:) y_constraints(1,:)]);
    grid on;
end
%%
filename = input('Enter file name: ', 's');
if length(filename) == 0
    disp('WARNING! Nothing saved');
    return;
end

if ispc == 1
    save(strcat(pwd, '\maps\' , filename));
else
    save(strcat(pwd, '/maps/' , filename));
end
