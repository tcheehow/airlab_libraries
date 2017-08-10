function [alpha, rL, rR] = line_estimator(points)

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
        if (i<=3)
            A = [A; x, -1, 0];
        else
            A = [A; x, 0, -1];
        end
        B = [B; -y];
    end
    
%     x = pinv(A'*A)*(A'*B);
%     x = (A'*A)\(A'*B);
    x = lsqr(A,B);
    alpha = atan(x(1));
    rL = x(2) * cos(alpha);
    rR = x(3) * cos(alpha);
    
end