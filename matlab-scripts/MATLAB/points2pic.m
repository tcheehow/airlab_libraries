function out = points2pic(points)

    xmax        = 5;
    ymax        = 10;
    resolution  = 0.1;
    xdim        = (xmax - (-xmax)) / resolution;
    ydim        = (ymax - (-ymax)) / resolution;
    out         = zeros(xdim, ydim, 'uint8');
    
    for i=1:length(points)
        x = points(i,1);
        y = points(i,2);
        x_index = (x + xmax) * (xdim) / (2 * xmax);
        x_index = round(x_index + 1);
        y_index = (y + ymax) * (xdim) / (2 * ymax);
        y_index = round(y_index + 1);
        out(x_index, y_index) = 255;
    end
    
end