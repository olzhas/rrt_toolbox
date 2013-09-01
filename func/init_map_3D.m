function [output, num] = init_map_3D();
close all;
clear;

prompt = {'Number of obstacles'};
dlg_title = 'Enter obstacles info';
def = {'3'};
num = str2double(inputdlg(prompt, dlg_title, 1, def));

output = cell(1,num);

collision = 1;
while (collision > 0)

    collision = 0;
    
    for ind = 1:num
        output{ind} = make_cube(rand(3, 1)*30, rand*3, rand*3, rand*3, rand*360, rand*360, rand*360);
    end

    for ind1 = 1:num-1
        for ind2 = (ind1 + 1):num
            if (obb_collision3D(output{ind1}, output{ind2}))
                collision = 1;
            end
        end
    end

end

close all;
figure(1);
hold on;
for ind = 1:num
    plot_c(output{ind}, 'r');
end

hold off;
view(3);
grid on;
axis equal;

d = questdlg('Do you want to save this configuration?', 'Yes', 'No');

if(strcmp(d, 'Yes') == 1)
    prompt = {'filename'};
    dlg_title = 'Save file';
    def = {'map.mat'};
    filename = inputdlg(prompt, dlg_title, 1, def);
    save(strcat(pwd, '\maps\' , filename{1}));
end

