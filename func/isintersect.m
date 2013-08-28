function result = isintersect(SHAPE, LINE, m_obs_array, b_obs_array, vert_num)
% isintersect(SHAPE, LINE)
% this function check whether we intersect the shape or not
% SHAPE could be polygon or line
% LINE is always line
%

result = 0;
m = (LINE(2, 2) - LINE(1, 2)) /  (LINE(2, 1) - LINE(1, 1));
b =  LINE(1, 2) - m * LINE(1, 1);
radius = 0.01;

% find min and max of a line
if LINE(1, 1) >= LINE(2, 1)
    x_edge_max = LINE(1, 1);
    x_edge_min = LINE(2, 1);
else
    x_edge_max = LINE(2, 1);
    x_edge_min = LINE(1, 1);
end

if LINE(1, 2) >= LINE(2, 2)
    y_edge_max = LINE(1, 2);
    y_edge_min = LINE(2, 2);
else
    y_edge_max = LINE(2, 2);
    y_edge_min = LINE(1, 2);
end

for k = 1:vert_num
    % y = m * x + b
    
    m_obs = m_obs_array(k);
    b_obs = b_obs_array(k);
    
    % consider this lines ???
    
%     if (m_obs + radius > m) && (m_obs - radius < m) && (b_obs + radius > b) && (b_obs - radius < b)
%        result = 1;
%        return;
%     end
    
    % I couldn't find better solution to deal this min max detection =)
    % this is the fastest one
    if SHAPE(k, 1) >= SHAPE(k+1, 1)
        x_max = SHAPE(k, 1);
        x_min = SHAPE(k+1, 1);
    else
        x_max = SHAPE(k+1, 1);
        x_min = SHAPE(k, 1);
    end
    
    if SHAPE(k, 2) >= SHAPE(k+1, 2)
        y_max = SHAPE(k, 2);
        y_min = SHAPE(k+1, 2);
    else
        y_max = SHAPE(k+1, 2);
        y_min = SHAPE(k, 2);
    end
    
    x_intersection = (b - b_obs)/(m_obs - m);
    y_intersection = m_obs * x_intersection + b_obs;

    if (x_intersection >= (x_min-radius) && x_intersection <= (x_max+radius)) ...
            && (x_intersection >= (x_edge_min-radius) && x_intersection <= (x_edge_max+radius)) ...
            && (y_intersection >= (y_edge_min-radius) && y_intersection <= (y_edge_max+radius)) ...
            && (y_intersection >= (y_min-radius) && y_intersection <= (y_max+radius))
        result = 1;
        return;
    end
end
