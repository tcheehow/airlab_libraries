function out = rpy(orient,xyzs)

if (nargin < 2)
    % cartesian input from the polar tof readings
    xyzs       = [1 0 0 1]';
    
    yaw     = pi/4;
    pitch   = pi/4;
    roll    = pi/4;
else
    yaw     = orient(1);
    pitch   = orient(2);
    roll    = orient(3);
    
end



% default is 'zyx' => yaw, pitch, roll
%     rot_roll        = eul2rotm([0, 0, roll], 'zyx');
%     rot_pitch       = eul2rotm([0, pitch, 0], 'zyx');
%     r1              = rot_roll * r;
%     r2              = rot_pitch * r1;
%     rotm            = rot_pitch * rot_roll

%     rotm            = eul2rotm([yaw, pitch, roll], 'zyx');

% use homogenenous transformation instead
rotm            = eul2tform([yaw, pitch, roll], 'zyx');
rotm(:,end)     = xyzs;

out             = rotm;
%     out             = rotm * r;
end