function show_map_circular( map_name )
%SHOW_MAP Stupid function to show maps with obstacles
% show_map_circular( map_name )

close all;
figure;
load ([pwd, '\maps\',  map_name]);
hold on;

t = 0:0.001:2*pi;

for obs_ind = 1:num
    x = output{obs_ind}(1);
    y = output{obs_ind}(2);
    r = output{obs_ind}(3);
    
    cir_x = r*cos(t) + x;
    cir_y = r*sin(t) + y;
    s2p2 =  fill(cir_x, cir_y, 'r');
    set(s2p2,'HandleVisibility','off','EdgeAlpha',0);
end

grid on; axis square; axis([x_constraints y_constraints]);
title('Rapidly Exploring Random Tree','FontWeight','Demi');
xlabel(' Position (m) ' , 'FontWeight', 'Demi');
ylabel(' Position (m) ' , 'FontWeight', 'Demi');
hold off;
end

