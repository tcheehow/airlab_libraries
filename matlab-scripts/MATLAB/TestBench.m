function out = TestBench()
close
clear all

ranges = []; %list of polar coordinates (r, theta)
ranges = [ranges; rand, pi/4];
ranges = [ranges; rand, 3*pi/4];
ranges = [ranges; rand, -3*pi/4];
ranges = [ranges; rand, -pi/4];

%(2) (1)
%(3) (4)

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

%% ROS Subscriber


laser1 = rossubscriber('/teraranger1/laser/scan');
laser2 = rossubscriber('/teraranger2/laser/scan');
laser3 = rossubscriber('/teraranger3/laser/scan');
laser4 = rossubscriber('/teraranger4/laser/scan');
laser5 = rossubscriber('/teraranger5/laser/scan');
laser6 = rossubscriber('/teraranger6/laser/scan');
    
%% Loop

while(true)
    
    v1      = receive(laser1);
    v2      = receive(laser2);
    v3      = receive(laser3);
    v4      = receive(laser4);
    v5      = receive(laser5);
    v6      = receive(laser6);
    
    [alpha, dX] = PosEstimator(v1.Ranges, v2.Ranges)
    
    ranges_c    = rotate(ros_input(v1, v2, v3, v4));

    ranges  = [v4.Ranges, 3*pi/4; ...
            v1.Ranges, pi/4; ...
            v3.Ranges, -pi/4; ...
            v2.Ranges, -3*pi/4; ...
            v4.Ranges, 3*pi/4;];
        
%     [sxy, centre] = COA(ranges);    
    
%     ranges_hc = rotate(ranges_h);
    
    
%     figure(1)
%     plot(score(:,1), score(:,2), '+')
%     xlabel('1st Principal Component')
%     ylabel('2nd Principal Component')
%     
%     figure(2)
%     pareto(explained)
%     xlabel('Principal Component')
%     ylabel('Variance Explained (%)')
%     biplot(coefforth(:,1:2),'scores',score(:,1:2));
%     axis([-1 1 -1 1])
    
  
    %% Plot Output
    
    figure(1)
    plot([ranges_c(:,1);ranges_c(1,1)], [ranges_c(:,2); ranges_c(1,2)], '--b');
    hold on
%     plot(sxy(:,2), sxy(:,3), 'ro', 0, 0, 'xk');
%     plot(centre(1), centre(2), 'bo', 0, 0, 'xk');
%     plot(centre(1), centre(2), 'ro', polygonCentre(1), polygonCentre(2), 'bo', 0, 0, 'xk');
%     legend('Boundary','COM(Discrete)', 'COM(Continuous)', 'UAV');
    
    plot([ranges_c(1,1), 0, ranges_c(3,1)], [ranges_c(1,2), 0, ranges_c(3,2)], 'k');
    plot([ranges_c(2,1), 0, ranges_c(4,1)], [ranges_c(2,2), 0, ranges_c(4,2)], 'k');
    %     text(double(centre(1)), double(centre(2)), '\leftarrow COM - Point Mass');
    %     text(double(polygonCentre(1)), double(polygonCentre(2)), '\leftarrow COM - Uniform Density');
    %     text(0, 0, '\leftarrow UAV');
%     axis([-14 14 -14 14])
%     xlabel(sprintf('COM(Discrete) %g', 0-centre(1)), 'FontSize', 8);
    %     xlabel(['\bold{COM(Discrete)}: ' num2str(0-centre(1))]);
    %     xlabel(['\bold{COM(Continuous)}: ' num2str(0-polygonCentre(1))]);
    hold off
    
end

end

