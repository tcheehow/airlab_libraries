function out = Simulator(  )

close all;
clear;

% uav body angles w.r.t world frame
yaw     = pi/8;
pitch   = pi/4;
roll    = 0;


%% sensor ray vectors definition in polar coordinates. sensor = [mag, direction]

sensor  = ...
    [1, -pi/4;
    1, -pi/2;
    1, -3*pi/4;
    1, 3*pi/4;
    1, pi/2;
    1, pi/4];

%% rotation of sensor vectors to cartesian

sensor_cart = rotate(sensor);
% figure()
% sensor_cart
% plot(-sensor_cart(:,2), sensor_cart(:,1));
% axis equal

%% rotate from body frame to world frame

% rotation matrix
rotm    = rpy(yaw, pitch, roll);

% rotate all sensor ray to world frame
for i=1:length(sensor_cart)
    v = sensor_cart(i,:)';
    v = rotm * v;
    sensor_cart(i,:) = v';
end

%     % sanity check
%     rotm    = rotm';
%     for i=1:length(sensor_cart)
%         v = sensor_cart(i,:)';
%         v = rotm * v;
%         sensor_cart(i,:) = v';
%     end

%% ray tracing for finding intersections

intersects = [];

% point on the plane
V0  = [0; 2.5; 0];
% normal vector to the plane
n   = [0; 2.5; 0];

% right tunnel wall intersect
for i=1:3
    P0          = [0;0;0];
    P1          = sensor_cart(i,:)';
    
    tmp         = PlaneLineIntersect(P0,P1,V0,n);
    intersects  = [intersects, tmp];
end

% left tunnel wall intersect
for i=4:6
    P0          = [0;0;0];
    P1          = sensor_cart(i,:)';
    
    tmp         = PlaneLineIntersect(P0,P1,-V0,n);
    intersects  = [intersects, tmp];
end

out = intersects

%%  Project the Vectors to Horizontal Plane

out_xy = out;
out_xy(end, :) = 0;

%%  Estimating Line Parameters
%   Using simulated terarangers reading

%     [alpha, rho] = line_estimator(ranges_c(4:6,:)); % for Method 2

% rotm    = rotm';


% % using world frame data
% ranges_c = out';

% rotate all sensor ray to world frame
% for i=1:length(ranges_c)
%     v = ranges_c(i,:);
%     v = rotm' * v';
%     ranges_c(i,:) = v';
% end


% using body frame data
ranges_c = out_xy';

rotm_yaw   = rpy(yaw, 0, 0);
% rotate all sensor ray to world frame
for i=1:length(ranges_c)
    v = ranges_c(i,:);
    v = rotm_yaw * v';
    ranges_c(i,:) = v';
end


% project it as if it is on the horizontal plane
% [theta, rho] = cart2pol(ranges_c(:,1), ranges_c(:,2));
% figure()
% polarplot(theta, rho);

[alpha, rhoL, rhoR] = line_estimator(ranges_c(:,1:2))
width = abs(rhoL) + abs(rhoR);
line_x = [-5:0.1:5];

figure()
plot(-[ranges_c(:,2);ranges_c(1,2)], [ranges_c(:,1); ranges_c(1,1)], '*-b');
hold on

plot(-(rhoL-line_x*sin(alpha)) / cos (alpha), line_x, '--g');
plot(-(rhoR-line_x*sin(alpha)) / cos (alpha), line_x, '--g');

% plot(-(rhoL-line_x*sin(alpha)) / cos (alpha), line_x, '--g');
% plot(-(rhoR-line_x*sin(alpha)) / cos (alpha), line_x, '--g');

% plot(line_x, (rhoL-line_x*cos(alpha)) / sin (alpha), '--g');
% plot(line_x, (rhoR-line_x*cos(alpha)) / sin (alpha), '--g');
%     plot(v0(1), v0(2), 'ok');

limit = 1.2*max(max(abs(intersects)));

centre = -(width/2)-rhoR;
plot(centre, 0, 'xk');
axis([-limit limit -limit limit])
alpha     = rad2deg(alpha)
xlabel({['centre: ' num2str(centre)], ['yaw error: ' num2str(alpha)]});
title('Simulated TeraRangers Reading on Body Frame');
hold off

%% visualisation of matlab simulator

% right line
figure();
plot3(out(1,1:3), out(2,1:3), out(3,1:3), '*-k');
hold on;

% left line
plot3(out(1,4:6), out(2,4:6), out(3,4:6), '*-k');

% ray
for i=1:length(sensor_cart)
    plot3([0 out(1,i)], [0 out(2,i)], [0, out(3,i)], '*--b');
end

% right line (projected)
plot3(out_xy(1,1:3), out_xy(2,1:3), out_xy(3,1:3), '*-r');
hold on;

% left line (projected)
plot3(out_xy(1,4:6), out_xy(2,4:6), out_xy(3,4:6), '*-r');

% ray (projected)
for i=1:length(sensor_cart)
    plot3([0 out_xy(1,i)], [0 out_xy(2,i)], [0, out_xy(3,i)], '*--r');
end

% rotm_   = rpy(yaw, pitch, roll);
heading = rotm * [1 0 0]'
% uav heading
%     heading = rotate([3, yaw+pi/2]);
plot3([0, heading(1)], [0 heading(2)], [0, heading(3)], '+-g');

% check the UAV heading
uav_heading = atand(heading(2)/heading(1))

xlim([-limit limit])
ylim([-limit limit])
zlim([-limit limit])

% plot tunnel surface
[X, Z]  = meshgrid(-limit:1:limit, -limit:1:limit);
Y       = 2.5 * ones(size(X));
CO(:,:,1) = zeros(size(Z));
CO(:,:,2) = zeros(size(Z));
CO(:,:,3) = zeros(size(Z));
surf(X,Y,Z,CO,'EdgeColor','none','FaceAlpha',0.2);
surf(X,-Y,Z,CO,'EdgeColor','none','FaceAlpha',0.2);
yaw     = rad2deg(yaw)
title('Simulated TeraRangers Reading on World Frame');
xlabel({['yaw error: ' num2str(yaw)]});
hold off;

end

function cart = rotate(polar)
d = size(polar);
r = polar(:,1);
th = polar(:,2);
x = r.*cos(th);
y = r.*sin(th);
cart = [x, y, zeros(d(1),1)];
% cart = [-y, x, zeros(d(1),1)];
end