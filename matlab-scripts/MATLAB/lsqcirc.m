clear
close all

% generate ground truth
centrex = 0.5;
centrey = 0.5;
r = 2;

% ground truth coord
th = 0:pi/20:2*pi;
x  = r * cos(th) + centrex;
y  = r * sin(th) + centrey;

N       = 4
std     = 0.05
% simulated from random pick
r_th    = (2*pi) * rand(1, N);
r_th    = sort(r_th);
r_x     = r * cos(r_th) + centrex;
r_x     = r_x + std*randn(1,N);
r_y     = r * sin(r_th) + centrey;
r_y     = r_y + std*randn(1,N)

[dx, dy, dr] = circle_estimator([r_x' r_y']);
% A = [2*r_x', 2*r_y', ones(3,1)];
% B = (r_x').^2 + (r_y').^2;
lsq_x  = dr * cos(th) + dx;
lsq_y  = dr * sin(th) + dy;
% 
figure()
plot(x, y, 'k-', centrex, centrey, 'ko')
axis equal

% random pick
hold on
plot(r_x, r_y, 'r*', centrex, centrey, 'ro', lsq_x, lsq_y, 'b-')
axis equal
hold off

