function Simulator_Body()

close all;
clear;

gazebo = true;
niceplots = false;

if (gazebo)
    rosshutdown;
    rosinit;
    laser1 = rossubscriber('/teraranger1/laser/scan');
    laser2 = rossubscriber('/teraranger2/laser/scan');
    laser3 = rossubscriber('/teraranger3/laser/scan');
    laser4 = rossubscriber('/teraranger4/laser/scan');
    laser5 = rossubscriber('/teraranger5/laser/scan');
    laser6 = rossubscriber('/teraranger6/laser/scan');
    model_states = rossubscriber('/gazebo/model_states');
    
    lsq_h = figure()
    
    yaw_acc = [];
    alpha_acc = [];
    com_h = figure()
    
else
    
    %% initialize uav instance with body frame angles and offset
    
    yaw     = pi/6;    pitch   = pi/4;    roll    = pi/4;
    
    uav_pos = [0;1;0;1]; % offset of the UAV = [x, y, z, s]  homogeneous coordinate
    
    uav     = FakeUAV([yaw,pitch,roll], uav_pos);
    
    %% get trone ray vector in world frame
    
    sensor_cart = uav.getSensorXY_World(); % uav trone ray vector in world frame
    
    %% get trone ranges by intersecting ray vector and tunnel surface
    
    % point on the plane
    V0          = [0; 2.5; 0];
    
    % normal vector to the plane
    n           = [0; 2.5; 0];
    
    intersects  = getRangesXY(uav, V0, n); %this solution is in the body frame
    dis         = getRanges(uav, intersects);
    
    
end

while (true)
    
    if(gazebo)
        
        states = receive(model_states);
        v1 = receive(laser1);
        v2 = receive(laser2);
        v3 = receive(laser3);
        v4 = receive(laser4);
        v5 = receive(laser5);
        v6 = receive(laser6);
        
        quat = states.Pose(3).Orientation;
        quat = [quat.W, quat.X, quat.Y, quat.Z];
        
        xyz0 = states.Pose(3).Position;
        xyz0 = [xyz0.X, xyz0.Y, xyz0.Z];
        
        eulZYX = quat2eul(quat);
        yaw = eulZYX(1);
        pitch = eulZYX(2);
        roll = eulZYX(3);
        
        dis = ros_input(v1, v2, v3, v4, v5, v6, false);
        
        %     plot(yaw); hold on;
        %     plot(pitch); hold on;
        %     plot(roll); hold off;
        %
        %     yaw = [yaw, eulZYX(1)];
        %     pitch = [pitch, eulZYX(2)];
        %     roll = [roll, eulZYX(3)];
        %     legend('yaw', 'pitch', 'roll')
        
        %% Project the Vectors to Horizontal Plane (Body-Fixed Rotation) - Body Frame
        
        [dis_x, dis_y]  = pol2cart(dis(:,2), dis(:,1)); %convert to cartersian
        dis_c           = [dis_x, dis_y, zeros(size(dis_x))];
        dis_c           = cart2hom(dis_c); %convert to homogeneous coordinate
        
        bodyXYZ     = [[1;0;0], [0;1;0], [0;0;1]];
        rotmZ       = eul2rotm([0,0,0], 'ZYX');
        bodyX1Y1Z1	= rotmZ * bodyXYZ;
        rotmY       = eul2rotm([0,-pitch,0], 'ZYX');
        bodyX2Y2Z2  = rotmY * bodyX1Y1Z1;
        rotmX       = eul2rotm([0,0,-roll], 'ZYX');
        bodyX3Y3Z3  = rotmX * bodyX2Y2Z2;
        x3y3z3      = [];
        
        % project all the body frame sensors to the horizontal plane
        for i = 1:length(dis_c)
            [x_cap, flag, relres, iter]   = lsqr(bodyX3Y3Z3(:,1:2), dis_c(i,1:3)', 1e-10);
            p       = bodyX3Y3Z3(:,1:2) * x_cap;
            x3y3z3  = [x3y3z3, p];
        end
        
        %% Project the Vectors to Horizontal Plane (Body-Fixed Rotation) - Body Frame
        
        % rotate to world frame using pitch and roll only. yaw and pos are not
        % known.
        
        rotm            = eul2tform([0, pitch, roll], 'zyx');
        
        x3y3z3 = cart2hom(x3y3z3');
        
        % rotate all sensor ray to world frame
        for i=1:length(x3y3z3)
            v = x3y3z3(i,:)';
            v = rotm * v;
            x3y3z3(i,:) = v';
        end
        
        % rotate all sensor ray to world frame
        for i=1:length(dis_c)
            v = dis_c(i,:)';
            v = rotm * v;
            dis_c(i,:) = v';
        end
        
        x3y3z3 = x3y3z3';
        
        if (niceplots)
            %% visualisation in body-fixed frame
            
            limit1 = max(max(abs(dis_c)));
            limit2 = max(max(abs(x3y3z3)));
            limit = 1.2*max([limit1, limit2]);
            h1 = visualise(dis_c, x3y3z3, limit, 'Simulated TeraRangers Reading on Body Frame');
            
            %% visualisation of matlab simulator (in world frame)
            
            h2 = visualise(dis_c, x3y3z3, limit, 'Simulated TeraRangers Reading on World Frame');
        end


        %%  Estimating Line Parameters
        %   Using simulated terarangers reading as seen from observator frame of
        %   reference
        
        ranges_c = x3y3z3';
        
        limit = 1.2*max(max(abs(ranges_c)));
        
        [alpha, rhoL, rhoR] = line_estimator(ranges_c(:,1:2))
        width = abs(rhoL) + abs(rhoR);
        line_x = [-limit:0.1:limit];
        
        figure(lsq_h)
        % plot(-[ranges_c(:,2);ranges_c(1,2)], [ranges_c(:,1); ranges_c(1,1)], '*--b');
        plot([ranges_c(:,1); ranges_c(1,1)], [ranges_c(:,2);ranges_c(1,2)], '*--b');
        hold on
        
        plot(line_x, (rhoL-line_x*sin(alpha)) / cos (alpha), '--g');
        plot(line_x, (rhoR-line_x*sin(alpha)) / cos (alpha), '--g');
        
        % ray
        for i=1:length(ranges_c)
            plot([0 ranges_c(i,1)], [0 ranges_c(i,2)], '*--b');
        end
        
        centre = (width/2)-rhoR;
        plot(0, -centre, 'xk');
        axis([-limit limit -limit limit])
        alpha     = rad2deg(alpha)
        xlabel({['centre: ' num2str(centre)], ['yaw error: ' num2str(alpha)]});
        title('Simulated TeraRangers Reading on World Frame');
        hold off
        
        %% visualisation of matlab simulator (world frame) using ground truth
        
        %     limit = 1.2*max(max(abs(ranges_c)));
        %
        %     out             = intersects';
        %     out_xy          = out;
        %     out_xy(:,end)   = 0;
        %
        %     limit = 1.2*max(max(abs(out)));
        %
        %     h3 = visualise(out, out_xy', limit, 'Ground Truth TeraRangers Reading on World Frame');
        %
        %     % plot tunnel surface
        %     figure(h3); hold on;
        %     [X, Z]  = meshgrid(-limit:1:limit, -limit:1:limit);
        %     Y       = 2.5 * ones(size(X));
        %     CO(:,:,1) = zeros(size(Z));
        %     CO(:,:,2) = zeros(size(Z));
        %     CO(:,:,3) = zeros(size(Z));
        %     surf(X,Y,Z,CO,'EdgeColor','none','FaceAlpha',0.2);
        %     surf(X,-Y,Z,CO,'EdgeColor','none','FaceAlpha',0.2);
        %     yaw     = rad2deg(yaw)
        %     title('Simulated TeraRangers Reading Ground Truth');
        %     xlabel({['yaw error: ' num2str(yaw)]});
        %     hold off;
        
        %% Compare Estimated Yaw with Gazebo Yaw
        
        figure(com_h);
        yaw_acc = [yaw_acc, yaw];
        alpha_acc = [alpha_acc, deg2rad(alpha)];
        plot(yaw_acc); hold on;
        plot(alpha_acc); hold off;
        legend('yaw', 'estimated yaw');
        ylim([-pi/2 pi/2]);
        
    else
        %%  Project the Vectors to Horizontal Plane (Fixed Axis Rotation, NOT WORKING)
        
        %     [dis_x, dis_y]  = pol2cart(dis(:,2), dis(:,1)); %convert to cartersian
        %     dis_c           = [dis_x, dis_y, zeros(size(dis_x))];
        %     dis_c           = cart2hom(dis_c); %convert to homogeneous coordinate
        % %     rotm            = eul2tform([0, -pitch, -roll], 'zyx');
        % %     rotm(:,end)     = uav_pos;
        %
        %     xAxis_body      = [1 0 0];
        %     yAxis_body      = [0 1 0];
        %     [xAxis_world, yAxis_world] = rotatePlane(xAxis_body, yAxis_body, 0, -pitch, -roll, uav.uavPos)
        %     zAxis_world     = cross(xAxis_world(1:3), yAxis_world(1:3));
        %
        %     A_world         = [xAxis_world(1:3), yAxis_world(1:3)];
        %     x               = [];
        %
        %     % project all the body frame sensors to the horizontal plane
        %     for i = 1:length(dis_c)
        %         [x_cap, flag, relres, iter]   = lsqr(A_world, dis_c(i,1:3)', 1e-10);
        %         p       = A_world * x_cap;
        %         x       = [x, p];
        %     end
        
        %% Project the Vectors to Horizontal Plane (Body-Fixed Rotation)
        
        [dis_x, dis_y]  = pol2cart(dis(:,2), dis(:,1)); %convert to cartersian
        dis_c           = [dis_x, dis_y, zeros(size(dis_x))];
        dis_c           = cart2hom(dis_c); %convert to homogeneous coordinate
        
        bodyXYZ     = [[1;0;0], [0;1;0], [0;0;1]];
        rotmZ       = eul2rotm([0,0,0], 'ZYX');
        bodyX1Y1Z1	= rotmZ * bodyXYZ;
        rotmY       = eul2rotm([0,-pitch,0], 'ZYX');
        bodyX2Y2Z2  = rotmY * bodyX1Y1Z1;
        rotmX       = eul2rotm([0,0,-roll], 'ZYX');
        bodyX3Y3Z3  = rotmX * bodyX2Y2Z2;
        x3y3z3      = [];
        
        % project all the body frame sensors to the horizontal plane
        for i = 1:length(dis_c)
            [x_cap, flag, relres, iter]   = lsqr(bodyX3Y3Z3(:,1:2), dis_c(i,1:3)', 1e-10);
            p       = bodyX3Y3Z3(:,1:2) * x_cap;
            x3y3z3  = [x3y3z3, p];
        end
        
        %% visualisation in body-fixed frame
        limit = 1.2*max(max(abs(intersects)));
        h1 = visualise(dis_c, x3y3z3, limit, 'Simulated TeraRangers Reading on Body Frame');
        
        %% visualisation of matlab simulator (in world frame)
        
        % rotate to world frame using pitch and roll only. yaw and pos are not
        % known.
        
        rotm            = eul2tform([0, pitch, roll], 'zyx');
        
        x3y3z3 = cart2hom(x3y3z3');
        
        % rotate all sensor ray to world frame
        for i=1:length(x3y3z3)
            v = x3y3z3(i,:)';
            v = rotm * v;
            x3y3z3(i,:) = v';
        end
        
        % rotate all sensor ray to world frame
        for i=1:length(dis_c)
            v = dis_c(i,:)';
            v = rotm * v;
            dis_c(i,:) = v';
        end
        
        x3y3z3 = x3y3z3';
        
        h2 = visualise(dis_c, x3y3z3, limit, 'Simulated TeraRangers Reading on World Frame');
        
        %% Convert of TROne Frame of Reference
        
        ranges_c = x3y3z3';
        
        %%  Estimating Line Parameters
        %   Using simulated terarangers reading as seen from observator frame of
        %   reference
        
        limit = 1.2*max(max(abs(ranges_c)));
        
        [alpha, rhoL, rhoR] = line_estimator(ranges_c(:,1:2))
        width = abs(rhoL) + abs(rhoR);
        line_x = [-limit:0.1:limit];
        
        figure()
        % plot(-[ranges_c(:,2);ranges_c(1,2)], [ranges_c(:,1); ranges_c(1,1)], '*--b');
        plot([ranges_c(:,1); ranges_c(1,1)], [ranges_c(:,2);ranges_c(1,2)], '*--b');
        hold on
        
        plot(line_x, (rhoL-line_x*sin(alpha)) / cos (alpha), '--g');
        plot(line_x, (rhoR-line_x*sin(alpha)) / cos (alpha), '--g');
        
        % ray
        for i=1:length(ranges_c)
            plot([0 ranges_c(i,1)], [0 ranges_c(i,2)], '*--b');
        end
        
        centre = (width/2)-rhoR;
        plot(0, -centre, 'xk');
        axis([-limit limit -limit limit])
        alpha     = rad2deg(alpha)
        xlabel({['centre: ' num2str(centre)], ['yaw error: ' num2str(alpha)]});
        title('Simulated TeraRangers Reading on World Frame');
        hold off
        
        %% visualisation of matlab simulator (world frame) using ground truth
        
        limit = 1.2*max(max(abs(ranges_c)));
        
        out             = intersects';
        out_xy          = out;
        out_xy(:,end)   = 0;
        
        limit = 1.2*max(max(abs(out)));
        
        h3 = visualise(out, out_xy', limit, 'Ground Truth TeraRangers Reading on World Frame');
        
        % plot tunnel surface
        figure(h3); hold on;
        [X, Z]  = meshgrid(-limit:1:limit, -limit:1:limit);
        Y       = 2.5 * ones(size(X));
        CO(:,:,1) = zeros(size(Z));
        CO(:,:,2) = zeros(size(Z));
        CO(:,:,3) = zeros(size(Z));
        surf(X,Y,Z,CO,'EdgeColor','none','FaceAlpha',0.2);
        surf(X,-Y,Z,CO,'EdgeColor','none','FaceAlpha',0.2);
        yaw     = rad2deg(yaw)
        title('Simulated TeraRangers Reading Ground Truth');
        xlabel({['yaw error: ' num2str(yaw)]});
        hold off;
    end
    
end


end

function out = getRangesXY(uav, V0, n)

out  = [];
sensor_cart = uav.getSensorXY_World();
rPoints     = sensor_cart(1:3,1:3);
lPoints     = sensor_cart(4:6,1:3);

% UAV position
P0          = uav.uavPos(1:3);

% right tunnel wall intersect
for i=1:3
    P1          = rPoints(i,1:3)';
    tmp         = PlaneLineIntersect(P0,P1,-V0,n);
    out         = [out, tmp];
end

% left tunnel wall intersect
for i=1:3
    P1          = lPoints(i,1:3)';
    tmp         = PlaneLineIntersect(P0,P1,V0,n);
    out         = [out, tmp];
end

end

function out = getRanges(uav, rangesXY)

% convert to what to the terarangers see in polar-coord by calculating the
% difference between the 1) world-frame xy-intersect of the trone ray with the
% tunnel surface and 2) the absolute world-frame xy-pose of the uav

out = [];

for i=1:6
    out = [out; norm(-uav.uavPos(1:3)+rangesXY(:,i)), uav.sensorOrientation(i,2)];
end

end

function [projectedTROne_cart, originalTROne_cart] = projectSensors(sensors_polar)

% TODO

end

function [ei_new, ej_new] = rotatePlane(ei, ej, y, p, r, uavPos)

rotm            = eul2tform([y, p, r], 'zyx');
rotm(:,end)     = uavPos;

%     xAxis_body      = [1 0 0];
ei      = cart2hom(ei)';
ei_new  = rotm * ei;

%     yAxis_body      = [0 1 0];
ej      = cart2hom(ej)';
ej_new  = rotm * ej;

end

function h = visualise(actual_coord, projected_coord, limit, str)

h = figure();

% right line
plot3(actual_coord(1:3,1), actual_coord(1:3,2), actual_coord(1:3,3), '*-k');
hold on;

% left line
plot3(actual_coord(4:6,1), actual_coord(4:6,2), actual_coord(4:6,3), '*-k');

% ray
for i=1:length(actual_coord)
    plot3([0 actual_coord(i,1)], [0 actual_coord(i,2)], [0, actual_coord(i,3)], '*--b');
end

% right line (projected)
plot3(projected_coord(1,1:3), projected_coord(2,1:3), projected_coord(3,1:3), '*-r');


% left line (projected)
plot3(projected_coord(1,4:6), projected_coord(2,4:6), projected_coord(3,4:6), '*-r');

% ray (projected)
for i=1:length(projected_coord)
    plot3([0 projected_coord(1,i)], [0 projected_coord(2,i)], [0, projected_coord(3,i)], '*--r');
end

% plot axis
% plot3([0 bodyX3Y3Z3(1,1)], [0 bodyX3Y3Z3(2,1)], [0 bodyX3Y3Z3(3,1)]);
% plot3([0 bodyX3Y3Z3(1,2)], [0 bodyX3Y3Z3(2,2)], [0 bodyX3Y3Z3(3,2)]);
% plot3([0 bodyX3Y3Z3(1,3)], [0 bodyX3Y3Z3(2,3)], [0 bodyX3Y3Z3(3,3)]);

% rotm_   = rpy(yaw, pitch, roll);
% heading = rotm * [1 0 0 1]'
% uav heading
%     heading = rotate([3, yaw+pi/2]);
% plot3([uav.uavPos(1), heading(1)], [uav.uavPos(2) heading(2)], [uav.uavPos(3), heading(3)], '+-g');

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
title(str);
% xlabel({['yaw error: ' num2str(yaw)]});

end