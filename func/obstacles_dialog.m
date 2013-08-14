function [output, num] = obstacles_dialog(x_constraints, y_constraints)
% OBSTACLE_DIALOG user can draw obstacles here
% this dialog window gets information
% obstacles_dialog(x_constraints, y_constraints)

prompt = {'Number of obstacles'};
dlg_title = 'Enter obstacles info';
def = {'3'};
num = str2double(inputdlg(prompt, dlg_title, 1, def));

output = cell(1,num);

for ind_obs = 1: num;
    
    grid on; axis square;
    axis([x_constraints, y_constraints]);
    i = 1;
    points = zeros(100, 2);
    button = 1;
    % until we press middle button of the mouse
    while button ~= 2
        %% obstacle drawing
        hold on;
        for k = 1:ind_obs-1
            fill(output{k}(1:end, 1), output{k}(1:end, 2), 'r');
        end
        hold off;
        [x, y, button] = ginput(1);
        if(button ~= 2)
            points(i, :) = [x y];
            i = i + 1;
        end
        plot(points(1:(i-1), 1), points(1:(i-1), 2))
        axis([x_constraints, y_constraints]);
        grid on; axis square;
    end
    points(i, :) = points(1, : );
    
    plot(points(1:i, 1), points(1:i, 2))
    axis([x_constraints, y_constraints]);
    grid on; axis square;
    output(ind_obs) = {points(1:i, :)};
end

d = questdlg('Do you want to save this configuration?', 'Yes', 'No');

if(strcmp(d, 'Yes') == 1)
    prompt = {'filename'};
    dlg_title = 'Save file';
    def = {'map.mat'};
    filename = inputdlg(prompt, dlg_title, 1, def);
    save(strcat(pwd, '\maps\' , filename{1}));
end





