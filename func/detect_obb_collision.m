function [collision_detected] = detect_obb_collision(link_center, link_corner, link_axis)

% Algorithm used here for OBB collision detection is based on the algorithm
% described in the book Real-Time Collision Detection Christer Ericson
% Page 102, Figure 4.9
% A nice C++ implementation was also found at http://www.flipcode.com/archives/2D_OBB_Intersection.shtml
%
% Recommendation to improve speed
% Considering that the robot will be only moving incrementally, one might
% store the test (which detects no collision) and employ that test for the
% first time) in the next iteration
%
% Author: Atakan Varol
% Date: 7/27/2012

collision_detected = 1;
% Super Simple Test without projections on global x and y axis
min_link1_x = min( link_corner{1}(:,1) );
max_link2_x = max( link_corner{2}(:,1) );
if min_link1_x > max_link2_x
    collision_detected = 0;
    return;
end

max_link1_x = max( link_corner{1}(:,1) );
min_link2_x = min( link_corner{2}(:,1) );
if min_link2_x > max_link1_x
    collision_detected = 0;
    return;
end

min_link1_y = min( link_corner{1}(:,2) );
max_link2_y = max( link_corner{2}(:,2) );
if min_link1_y > max_link2_y
    collision_detected = 0;
    return;
end

max_link1_y = max( link_corner{1}(:,2) );
min_link2_y = min( link_corner{2}(:,2) );
if min_link2_y > max_link1_y
    collision_detected = 0;
    return;
end

% Text by primary object axis projections
% Test for Axis 1
delta_cent = link_center(1,:) - link_center(2,:);
for ind1 = 1:2
    dist1 = abs(sum( delta_cent.* link_axis{1,ind1}));
    
    r1 = abs( sum(  (link_corner{1}(1,:) - link_center(1,:)).* link_axis{1,ind1}));
    r21 = abs( sum( (link_corner{2}(1,:) - link_center(2,:)).* link_axis{1,ind1}));
    r22 = abs( sum( (link_corner{2}(2,:) - link_center(2,:)).* link_axis{1,ind1}));
    
    if dist1 > (r1+r21) && dist1 > (r1 + r22)
        collision_detected = 0;
        return;
    end
end

% Test for Axis 2
for ind1 = 1:2
    dist1 = abs(sum( delta_cent.* link_axis{2,ind1}));
    
    r1 = abs( sum( ( ( link_corner{2}(1,:) - link_center(2,:))).* link_axis{2,ind1}));
    r21 = abs(sum( ( link_corner{1}(1,:) - link_center(1,:)).* link_axis{2,ind1}));
    r22 = abs(sum( ( link_corner{1}(2,:) - link_center(1,:)).* link_axis{2,ind1}));
    
    if dist1 > (r1+r21) && dist1 > (r1 + r22)
        collision_detected = 0;
        return;
    end
end

end