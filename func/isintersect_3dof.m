function result = isintersect_3dof(SHAPE, LINE)
% isintersect(SHAPE, LINE)
% this function check whether we intersect the shape or not
% SHAPE could be polygon or line
% LINE is always line
%

vert_num = size(SHAPE(:,1),1);
result = 0;

m = (LINE(2, 2) - LINE(1, 2)) /  (LINE(2, 1) - LINE(1, 1));
b =  LINE(1, 2) - m * LINE(1, 1);
radius = 0.1;

for k = 1:vert_num-1
    % y = m * x + b
    
    m_obs = (SHAPE(k, 2) - SHAPE(k+1, 2)) / (SHAPE(k, 1) - SHAPE(k+1, 1));
    b_obs = SHAPE(k, 2) - m_obs * SHAPE(k, 1);
    
    % consider this lines ???
    
    if (m_obs + radius > m) && (m_obs - radius < m) && (b_obs + radius > b) && (b_obs - radius < b)
        result = 1;
        return;
    end
    
    % ???
    
    x_intersection = (b - b_obs)/(m_obs - m);
    
    x_max = max (SHAPE(k, 1),SHAPE(k+1, 1));
    x_min = min (SHAPE(k, 1), SHAPE(k+1, 1));
    
    y_max = max (SHAPE(k, 2), SHAPE(k+1, 2));
    y_min = min (SHAPE(k, 2), SHAPE(k+1, 2));
    
    x_edge_max = max(LINE(1, 1), LINE(2, 1));
    x_edge_min = min(LINE(1, 1), LINE(2, 1));
    
    y_edge_max = max(LINE(1, 2), LINE(2, 2));
    y_edge_min = min(LINE(1, 2), LINE(2, 2));
    
    y_intersection = m_obs * x_intersection + b_obs;
    
    if ((x_intersection >= (x_min-radius) && x_intersection <= (x_max + radius)) ...
            && (x_intersection >= (x_edge_min - radius) && x_intersection <= (x_edge_max+radius)) ...
            && y_intersection<= y_edge_max + radius && y_intersection >= y_edge_min - radius ...
            && y_intersection<= y_max+radius && y_intersection >= y_min - radius)
        result = 1;
        return;
    end
end
