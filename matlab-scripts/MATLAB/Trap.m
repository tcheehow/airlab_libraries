function out = Trap()
close
clear all

edison = false;

rosshutdown;

if (edison)
     rosinit('http://192.168.0.106:11311');
%     rosinit;
else
    rosinit;
end

ranges = []; %list of polar coordinates (r, theta)

%(6) (1)
%(5) (2)
%(4) (3)

%% Rotation Matrix to Cartesian

    function cart = rotate(polar)
        r = polar(:,1);
        th = polar(:,2);
        x = r.*cos(th);
        y = r.*sin(th);
        cart = [x, y];
    end

%% Distance

    function dist = distance(r1, r2)
        dx = r2(1) - r1(1);
        dy = r2(2) - r1(2);
        dist = hypot(dx,dy);
    end

% Sanity Check
r1 = [0,0];
r2 = [1,2];


%% Trapezoid Parameters

    function out = fake_coordinates()
        out = [0, 1;
            2,2;
            2,0;
            0,0];
    end

%% ROS Subscriber

if (edison)
    laser1 = rossubscriber('/teraranger0');
    laser2 = rossubscriber('/teraranger1');
    laser3 = rossubscriber('/teraranger2');
    laser4 = rossubscriber('/teraranger3');
    laser5 = rossubscriber('/teraranger4');
    laser6 = rossubscriber('/teraranger5'); 
else
    laser1 = rossubscriber('/teraranger1/laser/scan');
    laser2 = rossubscriber('/teraranger2/laser/scan');
    laser3 = rossubscriber('/teraranger3/laser/scan');
    laser4 = rossubscriber('/teraranger4/laser/scan');
    laser5 = rossubscriber('/teraranger5/laser/scan');
    laser6 = rossubscriber('/teraranger6/laser/scan');
    model_states = rossubscriber('/gazebo/model_states');  
end

%% Loop

sensor_count = 6;
scan_max = 1;
ranges = zeros(scan_max*sensor_count, 2);
scan = 0;

rrate = rosrate(100);

while(true)
    
    %% Retrieve Ground Truth
    
    if(edison == false)
        xy_0 = receive(model_states);
        v0 = [xy_0.Pose(3).Position.Y, xy_0.Pose(3).Position.X];
    end
    
    if (scan >= scan_max) 
        scan = 0;
    end
    
    v1 = receive(laser1);
    v2 = receive(laser2);
    v3 = receive(laser3);
    v4 = receive(laser4);
    v5 = receive(laser5);
    v6 = receive(laser6);
    
    j = (scan*6)+1;
    
%     ranges = ros_input(v1, v2, v3, v4);
    ranges(j:j+5, :) = ros_input(v1, v2, v3, v4, v5, v6, edison); 
    scan = scan + 1;
    
    ranges_c = rotate(ranges);
    
    
    %ranges_c = fake_coordinates();
    
%     ranges_h = [v6.Ranges, pi;
%         v4.Ranges, 3*pi/4;
%         v1.Ranges, pi/4;
%         v5.Ranges, 0;
%         v3.Ranges, -pi/4;
%         v2.Ranges, -3*pi/4];
    
%     ranges_hc = rotate(ranges_h);
    
    
    
    %lower left corner is [0,0]
    %ranges_c = ranges_c - ranges_c(3, :)
    
    %     plot([ranges_c(:,1);ranges_c(1,1)], [ranges_c(:,2); ranges_c(1,2)], '--b');
%     a = distance(ranges_c(1,:), ranges_c(4,:));
%     b = distance(ranges_c(2,:), ranges_c(3,:));
%     c = distance(ranges_c(1,:), ranges_c(2,:));
%     d = distance(ranges_c(3,:), ranges_c(4,:));
%     s = (1/2)*(a+b+c+d);
%     n = 4 * sqrt((s-a)*(s-b)*(s-b-c)*(s-b-d));
%     h = n / (2* (abs(b-a)))
    
    %% Centroid Finder
    
    % x_c = (b/2) + ((2*a + b) * ((c^2 - d^2) / (6 * (b^2 - a^2))))
    % y_c = (2*a + b) * n / (6 * (b^2 - a^2))
    % hold on
    % plot(x_c, y_c, 'bo');
    
    %% Compare to COM Method
    
%     centre = COM(ranges_c);
    
%     distance(centre, ranges_c(1,:))
%     distance(centre, ranges_c(2,:))
%     distance(centre, ranges_c(3,:))
%     distance(centre, ranges_c(4,:))
    
%     polygonCentre           = centroidPolygon(ranges_c);
%     grad_dy                 = ranges_c(2,2) - ranges_c(1,2)
%     grad_dx                 = ranges_c(2,1) - ranges_c(1,1)
%     yaw_error               = atan(grad_dy / grad_dx)
%     [acc, thetas, rhos]     = HoughT(ranges_c,0.05,1);
%     out                     = HoughLine(acc, thetas, rhos)
%     
%     figure(1)
%     imshow(acc);
%     out             = HoughLine(hough_params);
    %% Hough
    
%     pic             = points2pic(ranges_c);
%     pic             = pic';
%     [H,T,R] = hough(pic, 'RhoResolution', 1, 'Theta', -90:0.1:89.9);
%     figure(1)
%     imshow(H,[],'XData',T,'YData',R,...
%         'InitialMagnification','fit');
%     xlabel('\theta'), ylabel('\rho');
%     axis on, axis normal, hold on;
    
%     P  = houghpeaks(H,5,'threshold',ceil(0.3*max(H(:))))
%     P  = houghpeaks(H,4);
%     x = T(P(:,2)); y = R(P(:,1));
%     plot(x,y,'s','color','white');
%         hough_param     = HoughT(ranges_hc, 0.05, 0.05)
%     hold off;
%     
%     lines = houghlines(pic,T,R,P,'MinLength',4);
%     figure(2), imshow(pic), hold on
%     max_len = 0;
%     
%     xy_long = [];
%     
%     for k = 1:length(lines)
%         xy = [lines(k).point1; lines(k).point2];
%         plot(xy(:,1),xy(:,2),'LineWidth',2,'Color','green');
%         
%         % Plot beginnings and ends of lines
%         plot(xy(1,1),xy(1,2),'x','LineWidth',2,'Color','yellow');
%         plot(xy(2,1),xy(2,2),'x','LineWidth',2,'Color','red');
%         
%         % Determine the endpoints of the longest line segment
%         len = norm(lines(k).point1 - lines(k).point2);
%         if ( len > max_len)
%             max_len = len;
%             xy_long = xy;
%         end
%     end
    
%     plot(xy_long(:,1),xy_long(:,2),'LineWidth',2,'Color','cyan');
%     hold off;

    %% Estimating Line Parameters
    
%     [alpha, rho] = line_estimator(ranges_c(4:6,:)); % for Method 2


	[alpha, rhoL, rhoR] = line_estimator(ranges_c)
    width = abs(rhoL) + abs(rhoR)
    line_x = [-5:0.1:5];
    
    %% Plot Output
    if (edison)
        offset = [0, 0;
            0, 0;
            0, -400;
            0, -400;
            0, 0;
            0, 0];        
        ranges_c = ranges + offset;
        ranges_c = ranges_c / 1000;
    end
    figure(3)
    plot([ranges_c(:,1);ranges_c(1,1)], [ranges_c(:,2); ranges_c(1,2)], '*-b');
    hold on
    
    if(edison == false)
        plot(line_x, (rhoL-line_x*cos(alpha)) / sin (alpha), '--g');
        plot(line_x, (rhoR-line_x*cos(alpha)) / sin (alpha), '--g');
%     plot(v0(1), v0(2), 'ok');
        plot(-(width/2)-rhoR, 0, 'xk');
    end





%     plot(centre(1), centre(2), 'ro', polygonCentre(1), polygonCentre(2), 'bo', 0, 0, 'xk');
%     legend('Boundary','COM(Discrete)', 'COM(Continuous)', 'UAV');
    
%     plot([ranges_c(1,1), 0, ranges_c(3,1)], [ranges_c(1,2), 0, ranges_c(3,2)], 'k');
%     plot([ranges_c(2,1), 0, ranges_c(4,1)], [ranges_c(2,2), 0, ranges_c(4,2)], 'k');
    %     text(double(centre(1)), double(centre(2)), '\leftarrow COM - Point Mass');
    %     text(double(polygonCentre(1)), double(polygonCentre(2)), '\leftarrow COM - Uniform Density');
    %     text(0, 0, '\leftarrow UAV');
    axis([-14 14 -14 14])
%     xlabel(sprintf('COM(Discrete) %g \n COM(Continuous) %g', 0-centre(1), 0-polygonCentre(1)), 'FontSize', 8);
    %     xlabel(['\bold{COM(Discrete)}: ' num2str(0-centre(1))]);
    %     xlabel(['\bold{COM(Continuous)}: ' num2str(0-polygonCentre(1))]);
    hold off
    
    waitfor(rrate);
    
end

    function out = centroidPolygon(v)
        % close loop of vertices
        N = length(v)
        v  = [v; v(1, :)]
        centrex = 0;
        centrey = 0;
        area = 0;
        for i = 1:(N)
            xi = v(i,1)
            yi = v(i,2)
            xi1 = v(i+1,1)
            yi1 = v(i+1,2)
            f = (xi*yi1 - xi1*yi)
            centrex = centrex + (xi + xi1)*f;
            centrey = centrey + (yi + yi1)*f;
            area = area + f;
        end
        area = area/2
        kyi = 6*area;
        centrex = centrex/(kyi);
        centrey = centrey/(kyi);
        out = [centrex, centrey];
    end

end

