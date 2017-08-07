function out = rpy(r,yaw,pitch,roll)

    if (nargin < 4)
    % cartesian input from the polar tof readings
    r       = [1 0 0]';
    
    yaw     = 0;
    pitch   = 0;
    roll    = -pi/4;
    end

    eul     = [yaw, pitch, roll];
    % default is 'zyx' => yaw, pitch, roll
    rotm    = eul2rotm(eul);
    
    
    out     = rotm * r;
end