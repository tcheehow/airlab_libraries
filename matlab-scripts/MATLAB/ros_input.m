function ranges = ros_input(v1, v2, v3, v4, v5, v6, edison)

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
if (edison)
    ranges = [v1.Range_, pi/4;
        v2.Range_, 0;
        v3.Range_, -pi/4;
        v4.Range_, -3*pi/4;
        v5.Range_, pi;
        v6.Range_, 3*pi/4];
else
    ranges = [v1.Ranges, -pi/4;
        v2.Ranges, -pi/2;
        v3.Ranges, -3*pi/4;
        v4.Ranges, 3*pi/4;
        v5.Ranges, pi/2;
        v6.Ranges, pi/4];
end

end

