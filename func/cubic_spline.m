function [ yy ] = cubic_spline( x, y, xx)
%CUBIC_SPLINE Summary of this function goes here
%   Detailed explanation goes here

% assuming that arguments has same number of elements

yy = zeros(1, size(xx,2));
x_num = size(x,2);

b = zeros(x_num-3, 1);
A = zeros(x_num-3, x_num-1);
h = zeros(x_num-2, 1);

for ind = 1 : x_num-1
    h(ind) = x(ind+1) - x(ind);
end

for ind = 1 : x_num-2
    if(ind+1 < x_num)
        A(ind, ind:ind+2) = [h(ind), 2*(h(ind+1) + h(ind)), h(ind+1)];
        b(ind) = 3 * ((y(ind+2) - y (ind+1))/h(ind+1) - (y(ind+1) - y(ind))/ h(ind));
    end
end

A = A(1:end,2:end-1);

a_i = y(1:x_num-1)';
b_i = zeros(x_num-1, 1);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% solving the system
% @TODO
% redo it by myself
c_i = inv(A) * b;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
d_i = zeros(x_num-1, 1);

c_i = [0 c_i' 0];
for ind = 1:x_num-2
    d_i(ind) = (c_i(ind+1) - c_i(ind)) / (3 * h(ind+1));
    b_i(ind) = (y(ind+1) - y(ind)) / h(ind+1) - h(ind+1)/3 * (c_i(ind+1) + 2 * c_i(ind));
end
d_i(x_num-1) = -c_i(x_num-1) / (3 * h(x_num-1));
b_i(x_num-1) = (y(x_num) - y(x_num-1)) / h(x_num-1) - h(x_num-1)/3 * (2 * c_i(x_num-1));


coeff_ind = 1;
for ind = 1:size(xx,2)
    
    yy(ind) = a_i(coeff_ind) + b_i(coeff_ind) * (xx(ind) - x(coeff_ind)) + c_i(coeff_ind) * (xx(ind) - x(coeff_ind))^2 + d_i(coeff_ind) * (xx(ind) - x(coeff_ind)) ^ 3;
    
    if xx(ind) >= x(coeff_ind+1)
        coeff_ind = coeff_ind + 1;
    end
end



end

