function out = Simulator(  )

    close all;
    clear;
%SIMULATOR Summary of this function goes here
%   Detailed explanation goes here
    
    %define sensor ray directions
    
%     sensor  = ...
%         [1, pi/4;
%          1, 0;
%          1, -pi/4;
%          1, -3*pi/4;
%          1, pi;
%          1, pi/4];

    sensor  = ...
        [1, pi/4;
         1, 0;
         1, -pi/4];
     
    sensor_cart = rotate(sensor);
    
    %rotate from body frame to world frame
    
    yaw     = 0;
    pitch   = -pi/4;
    roll    = -pi/4;
    
    rotm    = rpy(yaw, pitch, roll);
    for i=1:length(sensor_cart)
        v = sensor_cart(i,:)';
        v = rotm * v;
        sensor_cart(i,:) = v'
    end
    
    rotm    = rotm';
    for i=1:length(sensor_cart)
        v = sensor_cart(i,:)';
        v = rotm * v;
        sensor_cart(i,:) = v'
    end
    
    intersects = [];
    
    % point on the plane
    V0  = [2.5; 0; 0];
    % normal vector to the plane
    n   = [-2.5; 0; 0];
    
    for i=1:length(sensor)
        P0          = [0;0;0];
        P1          = sensor_cart(i,:)';
        
        tmp         = PlaneLineIntersect(P0,P1,V0,n);
        intersects  = [intersects, tmp];
    end
    
    out = intersects;
    
    figure();
    plot3(out(1,:), out(2,:), out(3,:), '*-k');
    hold on;
    
    for i=1:length(sensor)
    plot3([0 out(1,i)], [0 out(2,i)], [0, out(3,i)], '*-b');
    end
    hold off;
    xlim([0 3])
    ylim([-3 3])
    zlim([-3 3])
    
    function cart = rotate(polar)
        d = length(polar);
        r = polar(:,1);
        th = polar(:,2);
        x = r.*cos(th);
        y = r.*sin(th);
        cart = [x, y, zeros(d,1)];
    end

end

