function Copy_of_Simulator_Body()

close all;
clear;

gazebo = false;
edison = true;
niceplots = false;
rosbag = false;

if (gazebo)
    
    sensorOffset = ...
        [0.2256, -0.1741 0;
        0.1739 -0.1915 0;
        -0.1739 -0.1915 0;
        -0.1739 0.1915 0;
        0.1739 0.1915 0;
        0.2256 0.1741 0];
      
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
        
elseif (edison)
        
    rosshutdown;
    rosinit('http://192.168.4.3:11311');
    
%     sensorOffset = ...
%         [0.2256, -0.1741 0;
%         0.1739 -0.1915 0;
%         -0.1739 -0.1915 0;
%         -0.1739 0.1915 0;
%         0.1739 0.1915 0;
%         0.2256 0.1741 0];
    
     sensorOffset = ...
        [0.2615, -0.154 0;
        0.213 -0.169 0;
        -0.213 -0.169 0;
        -0.213 0.169 0;
        0.213 0.169 0;
        0.2615 0.154 0];
    
    lasers = rossubscriber('/teraranger_hub_one');
    state_roll = rossubscriber('/roll');
    state_pitch = rossubscriber('/pitch');
    
    body_h = figure()
    lsq_h = figure()
           
elseif (rosbag)
    
    rosshutdown;
    rosinit('http://airlab:11311');
    
    sensorOffset = ...
        [0.2256, -0.1741 0;
        0.1739 -0.1915 0;
        -0.1739 -0.1915 0;
        -0.1739 0.1915 0;
        0.1739 0.1915 0;
        0.2256 0.1741 0];
    
    laser = rossubscriber('/teraranger_hub_one');
    
    lsq_h = figure()
    
    yaw_acc = [];
    alpha_acc = [];
    com_h = figure()
    
else
    
    rosshutdown;
    rosinit('http://192.168.4.3:11311');
    
    fake_local_pose = rospublisher('/mavros/local_position/pose', 'geometry_msgs/PoseStamped');
    
    %% initialize uav instance with body frame angles and offset
    
    yaw     = 0;    pitch   = 0.02365;    roll    = -0.01975;
    
    uav_pos = [0;0;0;1]; % offset of the UAV = [x, y, z, s]  homogeneous coordinate
    
    uav     = FakeUAV([yaw,pitch,roll], uav_pos);
    
    %% get trone ray vector in world frame
    
    sensor_cart = uav.getSensorXY_World(); % uav trone ray vector in world frame
    
    %% get trone ranges by intersecting ray vector and tunnel surface
    
    % point on the plane
    V0          = [0; 0.89; 0];
    
    % normal vector to the plane
    n           = [0; 0.89; 0];
    
    intersects  = getRangesXY(uav, V0, n); %this solution is in the body frame
    dis         = getRanges(uav, intersects);
    
%     fake_trhub = rospublisher('/teraranger_hub_one', 'teraranger_array/RangeArray');
%     rangeArray_msg = rosmessage(fake_trhub);
%     ranges = []
%     for i=1:length(dis)
%         ranges_msg = rosmessage('sensor_msgs/Range');
%         ranges_msg.Range_ = dis(i,1);
%         ranges = [ranges; ...
%             ranges_msg];
%     end
%    rangeArray_msg.Ranges = ranges;
    
    
    lsq_h = figure(1)
    surf_h = figure(2)
    
end

while (true)
    if(rosbag)
        vs = receive(laser);        
    
    elseif(gazebo)
        
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
        dis_c           = dis_c + sensorOffset;
        
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
        
        sensorOffset_Projected	= [];
        sensorOffset_h            = cart2hom(sensorOffset); 
        
        for i = 1:length(sensorOffset_h)
            [x_cap, flag, relres, iter]   = lsqr(bodyX3Y3Z3(:,1:2), sensorOffset_h(i,1:3)', 1e-10);
            p       = bodyX3Y3Z3(:,1:2) * x_cap;
            sensorOffset_Projected  = [sensorOffset_Projected, p];
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
            h1 = visualise(sensorOffset, sensorOffset_Projected, dis_c, x3y3z3, limit, 'Simulated TeraRangers Reading on Body Frame');
            
            %% visualisation of matlab simulator (in world frame)
            
            % h2 = visualise(dis_c, x3y3z3, limit, 'Simulated TeraRangers Reading on World Frame');
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
            plot([sensorOffset(i,1) ranges_c(i,1)], [sensorOffset(i,2) ranges_c(i,2)], '*--b');
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
        
    elseif(edison)
        vs = receive(lasers);
        pitch = receive(state_pitch);
        pitch = pitch.Data;
        roll = receive(state_roll);
        roll = roll.Data;
        
        v1 = vs.Ranges(1);
        v1.Range_ = sensorComp(v1.Range_, 1);
        v2 = vs.Ranges(2);
        v2.Range_ = sensorComp(v2.Range_, 2);
        v3 = vs.Ranges(3);
        v3.Range_ = sensorComp(v3.Range_, 3);
        v4 = vs.Ranges(4);
        v4.Range_ = sensorComp(v4.Range_, 4);
        v5 = vs.Ranges(5);
        v5.Range_ = sensorComp(v5.Range_, 5);
        v6 = vs.Ranges(6);
        v6.Range_ = sensorComp(v6.Range_, 6);
        
        dis = ros_input(v1, v2, v3, v4, v5, v6, true);
        
        [dis_x, dis_y]  = pol2cart(dis(:,2), dis(:,1)); %convert to cartersian
        dis_c           = [dis_x, dis_y, zeros(size(dis_x))];
        dis_c           = dis_c + sensorOffset;
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
        
        sensorOffset = sensorOffset;
        
        sensorOffset_Projected	= [];
        sensorOffset_h            = cart2hom(sensorOffset); 
        
        for i = 1:length(sensorOffset_h)
            [x_cap, flag, relres, iter]   = lsqr(bodyX3Y3Z3(:,1:2), sensorOffset_h(i,1:3)', 1e-10);
            p       = bodyX3Y3Z3(:,1:2) * x_cap;
            sensorOffset_Projected  = [sensorOffset_Projected, p];
        end  
        
        %% visualisation in body-fixed frame
        limit1 = max(max(abs(dis_c)));
            limit2 = max(max(abs(x3y3z3)));
            limit = 1.2*max([limit1, limit2]);
            body_h = visualise(body_h, sensorOffset, sensorOffset_Projected, dis_c, x3y3z3, limit, 'Simulated TeraRangers Reading on Body Frame');
            view(2)
            body_h = figure(body_h)
            hold on;
            plot([-2, 1, 2], [0.895, 0.895, 0.895])
            plot([-2, 1, 2], [-0.895, -0.895, -0.895])
            hold off;
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
        
%         h2 = visualise(uav.sensorOffsetWorld(:,1:3), sensorOffset', dis_c, x3y3z3, limit, 'Simulated TeraRangers Reading on World Frame');
        
        %% Convert of TROne Frame of Reference
        
        ranges_c = x3y3z3';
        
        %%  Estimating Line Parameters
        %   Using simulated terarangers reading as seen from observator frame of
        %   reference
        
        limit = 1.2*max(max(abs(ranges_c)));
        
        [alpha, rhoL, rhoR] = line_estimator(ranges_c(:,1:2))
        width = abs(rhoL) + abs(rhoR)
        line_x = [-limit:0.1:limit];
        
        figure(lsq_h)
        % plot(-[ranges_c(:,2);ranges_c(1,2)], [ranges_c(:,1); ranges_c(1,1)], '*--b');
%         plot([ranges_c(:,1); ranges_c(1,1)], [ranges_c(:,2);ranges_c(1,2)], '*--b');
        plot([ranges_c(1:3,1)], [ranges_c(1:3,2)], '*--b');       
        hold on
        plot([ranges_c(4:6,1)], [ranges_c(4:6,2)], '*--b');    
        
        plot(line_x, (rhoL-line_x*sin(alpha)) / cos (alpha), '--g');
        plot(line_x, (rhoR-line_x*sin(alpha)) / cos (alpha), '--g');
        
        uav_pos = [0;0;0;1];
        uav     = FakeUAV([alpha,pitch,roll], uav_pos);
        
        % ray
        for i=1:length(ranges_c)
            plot([uav.sensorOffsetWorld(i,1) ranges_c(i,1)], [uav.sensorOffsetWorld(i,2) ranges_c(i,2)], '*--b');
        end
        
        centre = (width/2)-rhoR;
        plot(0, -centre, 'xk');
        axis([-limit limit -limit limit])
        alphad  = rad2deg(alpha)
        xlabel({['centre: ' num2str(centre)], ['yaw error: ' num2str(alpha) ' rad'],['yaw error: ' num2str(alphad) ' deg']});
        title('Simulated TeraRangers Reading on World Frame');
        hold off
        
        %% visualisation of matlab simulator (world frame) using ground truth
        
%         limit = 1.2*max(max(abs(ranges_c)));
%         
%         out             = intersects';
%         out_xy          = out;
%         out_xy(:,end)   = 0;
%         
%         limit = 1.2*max(max(abs(out)));
%         
%         surf_h = visualise(surf_h, uav.sensorOffsetWorld(:,1:3), sensorOffset', out, out_xy', limit, 'Ground Truth TeraRangers Reading on World Frame');
%         
%         % plot tunnel surface
%         figure(surf_h); hold on;
%         [X, Z]  = meshgrid(-limit:1:limit, -limit:1:limit);
%         Y       = 2.5 * ones(size(X));
%         CO(:,:,1) = zeros(size(Z));
%         CO(:,:,2) = zeros(size(Z));
%         CO(:,:,3) = zeros(size(Z));
%         surf(X,Y,Z,CO,'EdgeColor','none','FaceAlpha',0.2);
%         surf(X,-Y,Z,CO,'EdgeColor','none','FaceAlpha',0.2);
% %         yaw     = rad2deg(yaw)
%         title('Simulated TeraRangers Reading Ground Truth');
%         xlabel({['yaw error: ' num2str(yaw)]});
%         hold off;

        %% Compare Estimated Yaw with Gazebo Yaw
        
%         figure(com_h);
%         yaw_acc = [yaw_acc, yaw];
%         alpha_acc = [alpha_acc, deg2rad(alpha)];
%         plot(yaw_acc); hold on;
%         plot(alpha_acc); hold off;
%         legend('yaw', 'estimated yaw');
%         ylim([-pi/2 pi/2]);
        
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
        dis_c           = dis_c + uav.sensorOffset;
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
        
        sensorOffset = uav.sensorOffset;
        
        sensorOffset_Projected	= [];
        sensorOffset_h            = cart2hom(sensorOffset); 
        
        for i = 1:length(sensorOffset_h)
            [x_cap, flag, relres, iter]   = lsqr(bodyX3Y3Z3(:,1:2), sensorOffset_h(i,1:3)', 1e-10);
            p       = bodyX3Y3Z3(:,1:2) * x_cap;
            sensorOffset_Projected  = [sensorOffset_Projected, p];
        end  
        
        %% visualisation in body-fixed frame
        limit1 = max(max(abs(dis_c)));
            limit2 = max(max(abs(x3y3z3)));
            limit = 1.2*max([limit1, limit2]);
%             h1 = visualise(sensorOffset, sensorOffset_Projected, dis_c, x3y3z3, limit, 'Simulated TeraRangers Reading on Body Frame');
        
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
        
%         h2 = visualise(uav.sensorOffsetWorld(:,1:3), sensorOffset', dis_c, x3y3z3, limit, 'Simulated TeraRangers Reading on World Frame');
        
        %% Convert of TROne Frame of Reference
        
        ranges_c = x3y3z3';
        
        %%  Estimating Line Parameters
        %   Using simulated terarangers reading as seen from observator frame of
        %   reference
        
        limit = 1.2*max(max(abs(ranges_c)));
        
        [alpha, rhoL, rhoR] = line_estimator(ranges_c(:,1:2))
        width = abs(rhoL) + abs(rhoR);
        line_x = [-limit:0.1:limit];
        
%         figure(lsq_h)
%         % plot(-[ranges_c(:,2);ranges_c(1,2)], [ranges_c(:,1); ranges_c(1,1)], '*--b');
% %         plot([ranges_c(:,1); ranges_c(1,1)], [ranges_c(:,2);ranges_c(1,2)], '*--b');
%         plot([ranges_c(1:3,1)], [ranges_c(1:3,2)], '*--b');       
%         hold on
%         plot([ranges_c(4:6,1)], [ranges_c(4:6,2)], '*--b');    
%         
%         plot(line_x, (rhoL-line_x*sin(alpha)) / cos (alpha), '--g');
%         plot(line_x, (rhoR-line_x*sin(alpha)) / cos (alpha), '--g');
%         
%         % ray
%         for i=1:length(ranges_c)
%             plot([uav.sensorOffsetWorld(i,1) ranges_c(i,1)], [uav.sensorOffsetWorld(i,2) ranges_c(i,2)], '*--b');
%         end
%         
%         centre = (width/2)-rhoR;
%         plot(0, -centre, 'xk');
%         axis([-limit limit -limit limit])
%         alpha  = rad2deg(alpha)
%         xlabel({['centre: ' num2str(centre)], ['yaw error: ' num2str(alpha)]});
%         title('Simulated TeraRangers Reading on World Frame');
%         hold off
        
        %% visualisation of matlab simulator (world frame) using ground truth
        
        limit = 1.2*max(max(abs(ranges_c)));
        
        out             = intersects';
        out_xy          = out;
        out_xy(:,end)   = 0;
        
        limit = 1.2*max(max(abs(out)));
        
        surf_h = visualise(surf_h, uav.sensorOffsetWorld(:,1:3), sensorOffset', out, out_xy', limit, 'Ground Truth TeraRangers Reading on World Frame');
        
        % plot tunnel surface
        figure(surf_h); hold on;
        [X, Z]  = meshgrid(-limit:1:limit, -limit:1:limit);
        Y       = 2.5 * ones(size(X));
        CO(:,:,1) = zeros(size(Z));
        CO(:,:,2) = zeros(size(Z));
        CO(:,:,3) = zeros(size(Z));
        surf(X,Y,Z,CO,'EdgeColor','none','FaceAlpha',0.2);
        surf(X,-Y,Z,CO,'EdgeColor','none','FaceAlpha',0.2);
%         yaw     = rad2deg(yaw)
        title('Simulated TeraRangers Reading Ground Truth');
        xlabel({['yaw error: ' num2str(yaw)]});
        hold off;
    end
    
%     send(fake_trhub, rangeArray_msg)
    
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
    P0          = uav.uavPos(1:3) + uav.sensorOffsetWorld(i,1:3)';
    P1          = rPoints(i,1:3)';
    tmp         = PlaneLineIntersect(P0,P1,-V0,n);
    out         = [out, tmp];
end

% left tunnel wall intersect
for i=1:3
    P0          = uav.uavPos(1:3) + uav.sensorOffsetWorld(i+3,1:3)';
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
    out = [out; norm(-uav.uavPos(1:3)-uav.sensorOffsetWorld(i,1:3)'+rangesXY(:,i)), uav.sensorOrientation(i,2)];
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

function h = visualise(handle, sensor_coord, projectSensor_coord, actual_coord, projected_coord, limit, str)

h = figure(handle);

view(2)

% right line
plot3(actual_coord(1:3,1), actual_coord(1:3,2), actual_coord(1:3,3), '*-k');
hold on;

% left line
plot3(actual_coord(4:6,1), actual_coord(4:6,2), actual_coord(4:6,3), '*-k');

% ray
for i=1:length(actual_coord)
    plot3([sensor_coord(i,1) actual_coord(i,1)], [sensor_coord(i,2) actual_coord(i,2)], [sensor_coord(i,3) actual_coord(i,3)], '*--b');
end

% right line (projected)
plot3(projected_coord(1,1:3), projected_coord(2,1:3), projected_coord(3,1:3), '*-r');


% left line (projected)
plot3(projected_coord(1,4:6), projected_coord(2,4:6), projected_coord(3,4:6), '*-r');

% ray (projected)
for i=1:length(projected_coord)
    plot3([projectSensor_coord(1,i) projected_coord(1,i)], [projectSensor_coord(2,i) projected_coord(2,i)], [projectSensor_coord(3,i) projected_coord(3,i)], '*--r');
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
hold off

end