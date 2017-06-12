function out = Trap()
close
clear all

ranges = []; %list of polar coordinates (r, theta)
ranges = [ranges; pi/4, 1];
ranges = [ranges; 3*pi/4, 2];
ranges = [ranges; -3*pi/4, 3];
ranges = [ranges; -pi/4, 4];

%(1) (4)
%(2) (3)

%% Rotation Matrix to Cartesian

function cart = rotate(polar)
    th = polar(:,1);
    r = polar(:,2);
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

% Sanity Check
r1 = [0,0];
r2 = [1,2];


%% Trapezoid Parameters

function out = fake_coordinates()
    out = [0, 1;
    2,2;
    2,0;
    0,0];     
end

%ranges_c = rotate(ranges);
ranges_c = fake_coordinates();


%lower left corner is [0,0]
%ranges_c = ranges_c - ranges_c(3, :)

plot([ranges_c(:,1);ranges_c(1,1)], [ranges_c(:,2); ranges_c(1,2)]);
a = distance(ranges_c(1,:), ranges_c(4,:));
b = distance(ranges_c(2,:), ranges_c(3,:));
c = distance(ranges_c(1,:), ranges_c(2,:));
d = distance(ranges_c(3,:), ranges_c(4,:));
s = (1/2)*(a+b+c+d);
n = 4 * sqrt((s-a)*(s-b)*(s-b-c)*(s-b-d));
h = n / (2* (abs(b-a)))

%% Centroid Finder

% x_c = (b/2) + ((2*a + b) * ((c^2 - d^2) / (6 * (b^2 - a^2))))
% y_c = (2*a + b) * n / (6 * (b^2 - a^2))
% hold on
% plot(x_c, y_c, 'bo');

%% Compare to COM Method

centre = COM(ranges_c);

distance(centre, ranges_c(1,:))
distance(centre, ranges_c(2,:))
distance(centre, ranges_c(3,:))
distance(centre, ranges_c(4,:))

polygonCentre = centroidPolygon(ranges_c);

hold on
plot(centre(1), centre(2), 'ro', polygonCentre(1), polygonCentre(2), 'bo');
axis equal
hold off

    function out = centroidPolygon(v)
        % close loop of vertices
        N = length(v)
        v  = [v; v(1, :)]
        centrex = 0;
        centrey = 0;
        area = 0;
        for i = 1:(N)
            xi = v(i,1)
            yi = v(i,2)
            xi1 = v(i+1,1)
            yi1 = v(i+1,2)
            f = (xi*yi1 - xi1*yi)
            centrex = centrex + (xi +xi1)*f;
            centrey = centrey + (yi +yi1)*f;
            area = area + f;
        end
        area = area/2
        kyi = 6*area;
        centrex = centrex/(kyi);
        centrey = centrey/(kyi);
        out = [centrex, centrey]               
    end

end

