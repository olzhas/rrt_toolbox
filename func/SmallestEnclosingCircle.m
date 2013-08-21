function [Xcenter, Ycenter, R] = SmallestEnclosingCircle(X, Y, Xouter, Youter)
% [Xcenter, Ycenter, R] = SmallestEnclosingCircle(X, Y)
%
% Purpose:
% Calculate the Smallest Enclosing Circle of a set of points
%
% Input:  
% X                       array with X coordinates of the points, size: 1 x nr_points
% Y                       array with Y coordinates of the points, size: 1 x nr_points
%
% Input used for recursive use only:
% Xouter                  array with 1, 2 or 3 X coordinates of outermost points, size: 1 x nr_outer_points
% Youter                  array with 1, 2 or 3 Y coordinates of outermost points, size: 1 x nr_outer_points
%
% Output:
% Xcenter                 X coordinate of smallest enclosing circle
% Ycenter                 Y coordinate of smallest enclosing circle
% R                       radius of smallest enclosing circle
%
% History: 
% 14-Dec-2006   creation by FSta
%               based on an example by Yazan Ahed (yash78@gmail.com),
%               who based his code on a Java applet by Shripad Thite (http://heyoka.cs.uiuc.edu/~thite/mincircle/)


%Initialize Xouter, Youter, nr_outer_points
if nargin < 4
    Xmin = min(X);
    Ymin = min(Y);
    size_max = max(max(X) - Xmin, max(Y) - Ymin);
    Xouter = NaN; %init
    Youter = NaN; %init
    nr_outer_points = 0;
else
    nr_outer_points  = size(Xouter, 2);
end; %if nargin < 4

switch nr_outer_points
    case 0
        Xcenter = (max(X) + Xmin) / 2;
        Ycenter = (max(X) + Ymin) / 2;
        R = 0;
    case 1
        Xcenter = Xouter(1);
        Ycenter = Youter(1);
        R = 0;
    case 2
        Xcenter = (Xouter(1) + Xouter(2)) / 2;
        Ycenter = (Youter(1) + Youter(2)) / 2;
        R = sqrt((Xouter(1) - Xcenter)^2 + (Youter(1) - Ycenter)^2);
    case 3
        Xcenter = (Xouter(3)^2 * (Youter(1) - Youter(2)) + (Xouter(1)^2 + (Youter(1) - Youter(2)) * (Youter(1) - Youter(3))) * (Youter(2) - Youter(3)) + Xouter(2)^2 * (-Youter(1) + Youter(3))) / (2 * (Xouter(3) * (Youter(1) - Youter(2)) + Xouter(1) * (Youter(2) - Youter(3)) + Xouter(2) * (-Youter(1) + Youter(3))));
        Ycenter = (Youter(2) + Youter(3)) / 2 - (Xouter(3) - Xouter(2)) / (Youter(3) - Youter(2)) * (Xcenter - (Xouter(2) + Xouter(3)) / 2);
        R = sqrt((Xouter(1) - Xcenter)^2 + (Youter(1) - Ycenter)^2);
        return;
end; %switch nr_outer_points

for i = 1:size(X, 2)
    if (X(i) - Xcenter)^2 + (Y(i) - Ycenter)^2 > R^2
            if isempty(intersect([Xouter' Youter'], [X(i)' Y(i)'], 'rows'))
                Xouter(nr_outer_points + 1) = X(i);
                Youter(nr_outer_points + 1) = Y(i);
                [Xcenter, Ycenter, R] = SmallestEnclosingCircle(X(1:i), Y(1:i), Xouter, Youter);
            end; %if isempty(intersect([Xouter Youter], [X(i) Y(i)], 'rows'))
    end; %if (X(i) - Xcenter)^2 + (Y(i) - Ycenter)^2 > R^2
end; %for i
return;
