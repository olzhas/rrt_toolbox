close all;clear;
R = 2;
L = 10;
W = 3;

count = 0;

vx = zeros(6561,1);
vy = vx;
wz = vx;
path = zeros(6561, 2);
path_norm = path;
for t1 = -1:0.25:1
    for t2 = -1:0.25:1
        for t3 = -1:0.25:1
            for t4 = -1:0.25:1
                count = count + 1;
                vx(count) = R/4*(t1 + t2 + t3 + t4);
                vy(count) = R/4*(-t1 + t2 + t3 - t4);
                wz(count) = R/(4*(L+W))*(-t1 +t2 - t3 + t4);
            end
        end
    end
end

figure(1);
hold on;
plot3(vx, vy, wz, 'b', 'LineWidth', 1);
grid on; view(3);

x_old=0;y_old=0;z_old=0;

vertex_x = [x_old + cosd(z_old)*L + sind(z_old)*W; x_old + cosd(z_old)*L - sind(z_old)*W; x_old - cosd(z_old)*L - sind(z_old)*W; x_old - cosd(z_old)*L + sind(z_old)*W; x_old + cosd(z_old)*L + sind(z_old)*W];
vertex_y = [y_old + sind(z_old)*L - cosd(z_old)*W; y_old + sind(z_old)*L + cosd(z_old)*W; y_old - sind(z_old)*L + cosd(z_old)*W; y_old - sind(z_old)*L - cosd(z_old)*W; y_old + sind(z_old)*L - cosd(z_old)*W];

% plot(vertex_x, vertex_y, 'b');
% grid on;
% axis equal;


for trial = 1:5
    min = 100000;
    x = rand*10-5;
    y = rand*10-5;
    z = rand*pi;
    
    for count = 1:6561
        dist = (x-vx(count))^2 + (y-vy(count))^2 + (z-wz(count))^2;
        if (dist < min)
            min = dist;
            min_ind = count;
        end
    end
    path(trial, 1) = x;
    path(trial, 2) = y;
    figure(1);
    plot3([x; vx(min_ind)], [y; vy(min_ind)], [z; wz(min_ind)], 'k', 'LineWidth', 5);
    grid on; view(3);axis equal;
    
    x = vx(min_ind) + x_old;
    y = vy(min_ind) + y_old;
    z = wz(min_ind)/pi*360 + z_old;
    z = mod(z+360, 360);
    
    path_norm(trial, 1) = x;
    path_norm(trial, 2) = y;
    
    figure(2);
    hold on;
    vertex_x = [x + cosd(z)*L + sind(z)*W; x + cosd(z)*L - sind(z)*W; x - cosd(z)*L - sind(z)*W; x - cosd(z)*L + sind(z)*W; x + cosd(z)*L + sind(z)*W];
    vertex_y = [y + sind(z)*L - cosd(z)*W; y + sind(z)*L + cosd(z)*W; y - sind(z)*L + cosd(z)*W; y - sind(z)*L - cosd(z)*W; y + sind(z)*L - cosd(z)*W];
    plot(vertex_x, vertex_y, 'LineWidth', 5);
    
    x_old = x;
    y_old = y;
    z_old = z;
    
end

% plot(path(:,1)/10, path(:, 2)/10, 'k', 'LineWidth', 3);
% plot(path_norm(:,1), path_norm(:, 2), 'g', 'LineWidth', 3);
hold off;
