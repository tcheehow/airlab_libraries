function out = rpy(yaw,pitch,roll)

    if (nargin < 3)
    % cartesian input from the polar tof readings
    r       = [1 0 0]';
    
    yaw     = 0;
    pitch   = 0;
    roll    = -pi/4;
    end

    eul     = [yaw, pitch, roll];
    % default is 'zyx' => yaw, pitch, roll
    rotm    = eul2rotm(eul);
    
    out     = rotm;
%     out     = rotm * r;
end