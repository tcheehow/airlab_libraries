function ranges = ros_input(v1, v2, v3, v4, v5, v6)

% laser1 = rossubscriber('/teraranger1/laser/scan');
% laser2 = rossubscriber('/teraranger2/laser/scan');
% laser3 = rossubscriber('/teraranger3/laser/scan');
% laser4 = rossubscriber('/teraranger4/laser/scan');
% 
% v1 = receive(laser1);
% v2 = receive(laser2);
% v3 = receive(laser3);
% v4 = receive(laser4);

% ranges = [v4.Ranges, 3*pi/4;
%     v1.Ranges, pi/4;
%     v3.Ranges, -pi/4;
%     v2.Ranges, -3*pi/4];

ranges = [v2.Ranges, -3*pi/4;
        v6.Ranges, pi;
        v4.Ranges, 3*pi/4;
        v1.Ranges, pi/4;
        v5.Ranges, 0;
        v3.Ranges, -pi/4];

end

