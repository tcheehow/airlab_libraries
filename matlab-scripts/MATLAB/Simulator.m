function out = Simulator(  )

close all;
clear;

%% initialize uav instance with body frame angles and offset

yaw     = 0;    pitch   = pi/8;    roll    = pi/8;

uav_pos = [0;0;0;1]; % offset of the UAV = [x, y, z, s]  homogeneous coordinate

uav     = FakeUAV([yaw,pitch,roll], uav_pos);

sensor  = uav.sensorOrientation; % sensor ray vectors definition in polar coordinates. sensor = [mag, direction]

%% rotation of sensor vectors to cartesian

sensor_cart = uav.getRayCartesian_body(); %convert to homogeneous coordinate

%% rotate from body frame to world frame

% rotation matrix
% rotm    = rpy([yaw, pitch, roll], uav.uavPos);
rotm    = uav.getRotMatrix();

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

intersects  = [];

% UAV position
P0          = uav_pos(1:3);

% point on the plane
V0          = [0; 2.5; 0];
% normal vector to the plane
n           = [0; 2.5; 0];
% n   = V0 - P0;

% right tunnel wall intersect
for i=1:3
    P1          = sensor_cart(i,1:3)';    
    tmp         = PlaneLineIntersect(P0,P1,V0,n);
    intersects  = [intersects, tmp];
end

% left tunnel wall intersect
for i=4:6
    P1          = sensor_cart(i,1:3)';    
    tmp         = PlaneLineIntersect(P0,P1,-V0,n);
    intersects  = [intersects, tmp];
end

out = intersects
dis = [];

%% convert to what to the terarangers see

for i=1:6
	dis = [dis; norm(-uav_pos(1:3)+out(:,i)), sensor(i,2)];
end

% polarplot(dis(:,2), dis(:,1))

%%  Project the Vectors to Horizontal Plane

ground_truth = true; %using simulated terarangers

if (ground_truth)
    out_xy  = out;
    out_xy  = out_xy - uav_pos(1:3);
    out_xy(end, :)    = 0;
else
    [dis_x, dis_y]  = pol2cart(dis(:,2), dis(:,1));
    dis_c           = [dis_x, dis_y, zeros(size(dis_x))];
    
    dis_c           = cart2hom(dis_c);
    rotm            = eul2tform([0, pitch, roll], 'zyx');
    rotm(:,end)     = uav_pos;
    
    normal_body     = [0 0 1];
    normal_body     = cart2hom(normal_body)'; 
    
    xAxis_body      = [1 0 0];
    xAxis_body      = cart2hom(xAxis_body)';
    xAxis_world     = rotm * xAxis_body;
    
    yAxis_body      = [0 1 0];
    yAxis_body      = cart2hom(yAxis_body)';
    yAxis_world     = rotm * yAxis_body;
    
    A_world         = [xAxis_world(1:3), yAxis_world(1:3)]; 
    x               = [];
    
    % rotate all sensor ray to world frame
    for i = 1:length(dis_c)
        x_cap   = lsqr(A_world, dis_c(i,1:3)');
        p       = A_world * x_cap;
        x       = [x, p];    
    end    
end

%% visualisation of matlab simulator

if (ground_truth == false)

limit = 1.2*max(max(abs(intersects)));
dis_c = dis_c';

% right line
figure();
plot3(dis_c(1,1:3), dis_c(2,1:3), dis_c(3,1:3), '*-k');
hold on;

% left line
plot3(dis_c(1,4:6), dis_c(2,4:6), dis_c(3,4:6), '*-k');

% ray
for i=1:length(sensor_cart)
    plot3([P0(1) dis_c(1,i)], [P0(2) dis_c(2,i)], [P0(3), dis_c(3,i)], '*--b');
end

% right line (projected)
plot3(x(1,1:3), x(2,1:3), x(3,1:3), '*-r');


% left line (projected)
plot3(x(1,4:6), x(2,4:6), x(3,4:6), '*-r');

% ray (projected)
for i=1:length(sensor_cart)
    plot3([P0(1) x(1,i)], [P0(2) x(2,i)], [P0(3), x(3,i)], '*--r');
end

% rotm_   = rpy(yaw, pitch, roll);
heading = rotm * [1 0 0 1]'
% uav heading
%     heading = rotate([3, yaw+pi/2]);
plot3([P0(1), heading(1)], [P0(2) heading(2)], [P0(3), heading(3)], '+-g');

% check the UAV heading
% uav_heading = atand(heading(2)/heading(1))

xlim([-limit limit])
ylim([-limit limit])
zlim([-limit limit])

% plot tunnel surface
% [X, Z]  = meshgrid(-limit:1:limit, -limit:1:limit);
% Y       = 2.5 * ones(size(X));
% CO(:,:,1) = zeros(size(Z));
% CO(:,:,2) = zeros(size(Z));
% CO(:,:,3) = zeros(size(Z));
% surf(X,Y,Z,CO,'EdgeColor','none','FaceAlpha',0.2);
% surf(X,-Y,Z,CO,'EdgeColor','none','FaceAlpha',0.2);
% yaw     = rad2deg(yaw)
% title('Simulated TeraRangers Reading on World Frame');
% xlabel({['yaw error: ' num2str(yaw)]});
hold off;

end


%% Convert of TROne Frame of Reference

% rotm    = rotm';

% % using world frame data
% ranges_c = out';

% rotate all sensor ray to world frame
% for i=1:length(ranges_c)
%     v = ranges_c(i,:);
%     v = rotm' * v';
%     ranges_c(i,:) = v';
% end

if (ground_truth)
    
    % using body frame data
    ranges_c = out_xy';
    ranges_c = cart2hom(ranges_c);
    
    rotm_yaw   = rpy([yaw, 0, 0], 0);
    % rotm_yaw   = rpy([yaw, 0, 0], -uav.uavPos);
    % rotate all sensor ray to world frame
    for i=1:length(ranges_c)
        v = ranges_c(i,:);
        v = rotm_yaw * v';
        ranges_c(i,:) = v';
    end
    
else
    
    ranges_c = dis_c;
    
end

%%  Estimating Line Parameters
%   Using simulated terarangers reading

%     [alpha, rho] = line_estimator(ranges_c(4:6,:)); % for Method 2

% project it as if it is on the horizontal plane
% [theta, rho] = cart2pol(ranges_c(:,1), ranges_c(:,2));
% figure()
% polarplot(theta, rho);
% 
% limit = 1.2*max(max(abs(ranges_c)));
% 
% [alpha, rhoL, rhoR] = line_estimator(ranges_c(:,1:2))
% width = abs(rhoL) + abs(rhoR);
% line_x = [-limit:0.1:limit];
% 
% figure()
% % plot(-[ranges_c(:,2);ranges_c(1,2)], [ranges_c(:,1); ranges_c(1,1)], '*--b');
% plot([ranges_c(:,1); ranges_c(1,1)], [ranges_c(:,2);ranges_c(1,2)], '*--b');
% hold on
% 
% plot(line_x, (rhoL-line_x*sin(alpha)) / cos (alpha), '--g');
% plot(line_x, (rhoR-line_x*sin(alpha)) / cos (alpha), '--g');
% 
% % ray
% for i=1:length(ranges_c)
%     plot([0 ranges_c(i,1)], [0 ranges_c(i,2)], '*--b');
% end
% 
% centre = (width/2)-rhoR;
% plot(0, -centre, 'xk');
% axis([-limit limit -limit limit])
% alpha     = rad2deg(alpha)
% xlabel({['centre: ' num2str(centre)], ['yaw error: ' num2str(alpha)]});
% title('Simulated TeraRangers Reading on Body Frame');
% hold off

%% visualisation of matlab simulator

limit = 1.2*max(max(abs(intersects)));

% right line
figure();
plot3(out(1,1:3), out(2,1:3), out(3,1:3), '*-k');
hold on;

% left line
plot3(out(1,4:6), out(2,4:6), out(3,4:6), '*-k');

% ray
for i=1:length(sensor_cart)
    plot3([P0(1) out(1,i)], [P0(2) out(2,i)], [P0(3), out(3,i)], '*--b');
end

% right line (projected)
plot3(out_xy(1,1:3), out_xy(2,1:3), out_xy(3,1:3), '*-r');


% left line (projected)
plot3(out_xy(1,4:6), out_xy(2,4:6), out_xy(3,4:6), '*-r');

% ray (projected)
for i=1:length(sensor_cart)
    plot3([P0(1) out_xy(1,i)], [P0(2) out_xy(2,i)], [P0(3), out_xy(3,i)], '*--r');
end

% rotm_   = rpy(yaw, pitch, roll);
heading = rotm * [1 0 0 1]'
% uav heading

plot3([P0(1), heading(1)], [P0(2) heading(2)], [P0(3), heading(3)], '+-g');


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

rotm

end