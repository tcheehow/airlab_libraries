function [acc, theta_bin, rho_bin] = HoughT(points, rho_resolution, theta_resolution)
    
    % rho and theta ranges
    rho_bin = -15:rho_resolution:15;
    theta_bin = deg2rad(-90:theta_resolution:90-theta_resolution);
    
    % Hough accumulator array of theta vs rho
    acc = zeros(length(rho_bin), length(theta_bin)); % row= rho, col=theta
    
    for i=1:length(points)
        
        x = points(i, 1);
        y = points(i, 2);
        
        % calculate rho.
        r = x * cos(theta_bin) + y * sin (theta_bin);
        
%         figure(5)
%         plot(theta_bin, r);
%         hold on;
        
        % voting
        % need to update the r into the accumulator
        for j = 1:length(r)
            row = search(r(j), rho_bin);
            col = j;
            acc(row, col) = acc(row, col) + 1; % 2 votes
        end
        
    end
%     hold off;
    
    %out_x = max(acc, [], 1);
    %out_y = max(acc, [], 2);
    %out = [out_x, out_y];
%     candidate = max(acc(:));
%     [I_row, I_column] = ind2sub(size(acc), find(acc==candidate));
%     r = rho_bin(I_row);
%     t = theta_bin(I_column);
%     out = [r', rad2deg(t')];
%     out = sort(out, 1);
    
    function row = search(r, bin)
        
        % fit r into the lowest closest rho_bin
        for k=1:length(bin)
          if (k < length(bin))
              if (r > bin(k) && r <= bin(k+1))
                  row = k;
              end
          else
              if (r > bin(k))
                row = k;
              end
          end
        end
        
    end

end