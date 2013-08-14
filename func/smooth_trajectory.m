function [ xx, yy, smooth_trajectory ] = smooth_trajectory( num_link, len_link,  link_pos_tree, link_ang_tree, backtrace_path, path_iter )
%SMOOTH_TRAJECTORY Smooth the initial trajectory to avoid jerked motion


joint_pos_states = zeros(num_link, 2, path_iter);
joint_ang_states = zeros(num_link, path_iter);

for ind = path_iter:-1:1
    joint_pos_states(:,:, ind) = link_pos_tree(:,:, backtrace_path(ind));
    joint_ang_states(:, ind) = link_ang_tree(:, backtrace_path(ind));
end

xx = 1:0.1:path_iter;
x = 1:1:path_iter;
yy = zeros(num_link, size(xx,2));
%yy2 = zeros(num_link, size(xx,2));
for temp_ind = 1:num_link
    yy(temp_ind, :) = cubic_spline(x, squeeze(joint_ang_states(temp_ind, path_iter:-1:1)), xx);
    %yy2(temp_ind, :) = interp1(x, squeeze(joint_ang_states(temp_ind, path_iter:-1:1)), xx, 'spline');
end

smooth_ang = zeros(num_link, size(xx,2));
smooth_trajectory = zeros(num_link, 2, size(xx,2));
for ind = 1:size(xx,2)
    smooth_ang(:,ind) = cumsum( yy(:,ind) );
    smooth_trajectory(:, 1, ind) = cumsum( len_link.*cos(smooth_ang(:,ind) ));
    smooth_trajectory(:, 2, ind) = cumsum( len_link.*sin(smooth_ang(:,ind) ));
end

end

