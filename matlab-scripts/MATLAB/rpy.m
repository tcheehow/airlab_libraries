function out = rpy(yaw,pitch,roll)

    if (nargin < 3)
    % cartesian input from the polar tof readings
    r       = [1 0 0]';
    
    yaw     = pi/6;
    pitch   = pi/4;
    roll    = pi/4;
    end

    % default is 'zyx' => yaw, pitch, roll
%     rot_roll        = eul2rotm([0, 0, roll], 'zyx');
%     rot_pitch       = eul2rotm([0, pitch, 0], 'zyx');
%     r1              = rot_roll * r;
%     r2              = rot_pitch * r1;
%     rotm            = rot_pitch * rot_roll
    
    rotm            = eul2rotm([yaw, pitch, roll], 'zyx');
    out             = rotm;
%     out             = rotm * r;
end