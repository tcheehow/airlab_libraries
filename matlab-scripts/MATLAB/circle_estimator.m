function [dx, dy, r] = circle_estimator(points)

    %% Method 1

%     xy_bar = mean(points);
%     x_bar = xy_bar(1);
%     y_bar = xy_bar(2);
%     
%     num = 0;
%     den = 0;
%     
%     for i=1:length(points)
%         x = points(i,1);
%         y = points(i,2);
%         num = num + (y_bar - y) * (x_bar - x);
%         den = den + ((y_bar - y)^2 - (x_bar - x)^2);
%     end
%     
%     tan2alpha = -2 * num / den;
%     alpha = atan(tan2alpha);
%     r = x_bar * cos(alpha) + y_bar * sin(alpha);
    
    %% Method 2
    
%     A = [];
%     B = [];
%     
%     for i=1:length(points)
%         x = points(i,1);
%         y = points(i,2);
%         A = [A; y, -1];
%         B = [B; -x];
%     end
%     
%     x = (A'*A)\(A'*B);
%     alpha = atan(x(1))
%     r = x(2) * cos(alpha)
    
    %% Method 3
    
    A = [];
    B = [];
    
    for i=1:length(points)
        x = points(i,1);
        y = points(i,2);
%         y = points(i,1);
%         x = points(i,2);
        A = [A; 2*x, 2*y, 1];
        B = [B; x*x + y*y];
    end
    
%     x = pinv(A'*A)*(A'*B);
%     x = (A'*A)\(A'*B);
    x = lsqr(A,B);
%     x = inv(A)*B;
    dx = x(1);
    dy = x(2);
    r = sqrt(x(3)+dx^2+dy^2);
    
end