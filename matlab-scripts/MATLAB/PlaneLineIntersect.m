function out = PlaneLineIntersect(P0, P1, V0, n)
    
%     %two points on the line
%     P0  = [0; 0; 0];
%     P1  = [sqrt(2);sqrt(2); 0];
    
%     
%     % point on the plane
%     V0  = [2.5; 0; 0];
%     % normal vector to the plane
%     n   = [-2.5; 0; 0];
    
    u   = P1 - P0;
    w   = V0 - P0;
    
    % check for line // to plane
    ret = dot(u, n);
    tol = 10e-13;
    
    if (abs(ret) <= tol) %when line is // to plane
        si = 0;
    else  
        si  = dot(n,w) / dot(n,u);
    end
    
    out = P0 + si * u;
    
end