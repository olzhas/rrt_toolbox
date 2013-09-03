function [ output_args ] = show_map( map_name )
%SHOW_MAP Stupid function to show maps with obstacles
% show_map( map_name ) 

figure
load ([pwd, '\maps\',  map_name]);
hold on;
for k = 1:num
    p2 = fill(output{k}(1:end, 1), output{k}(1:end, 2), 'r');
    set(p2,'HandleVisibility','off','EdgeAlpha',0);
end
grid on; axis square; axis([-20 20 -20 20]);
title('Rapidly Exploring Random Tree','FontWeight','Demi');
xlabel(' Position (m) ' , 'FontWeight', 'Demi');
ylabel(' Position (m) ' , 'FontWeight', 'Demi');
hold off;
end

