function out = rpy(yaw,pitch,roll)

    if (nargin < 3)
    % cartesian input from the polar tof readings
    r       = [0 0 0]';
    
    yaw     = 0;
    pitch   = pi/4;
    roll    = pi/4;
    end

    eul     = [yaw, roll, pitch];
    % default is 'zyx' => yaw, pitch, roll
    rotm    = eul2rotm(eul, 'zyx');
    
    out     = rotm;
    out     = rotm * r;
end