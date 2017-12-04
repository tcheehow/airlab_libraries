%% Import data from text file.
% Script for importing data from the following text file:
%
%    C:\Users\Chee How\Desktop\Logs\Flight002 (Good)\log003_vehicle_local_position_0.csv
%
% To extend the code to different selected data or a different text file,
% generate a function instead of a script.

% Auto-generated by MATLAB on 2017/10/04 11:48:27

clear
close all

%% Convert ULG to CSV
% execute pyulog logxxx.ulg

%% Initialize variables.
filename = 'Logs/sess040/log004_vehicle_local_position_0.csv';
delimiter = ',';
startRow = 2;

%% Format for each line of text:
%   column1: double (%f)
%	column2: double (%f)
%   column3: double (%f)
%	column4: double (%f)
%   column5: double (%f)
%	column6: double (%f)
%   column7: double (%f)
%	column8: double (%f)
%   column9: double (%f)
%	column10: double (%f)
%   column11: double (%f)
%	column12: double (%f)
%   column13: double (%f)
%	column14: double (%f)
%   column15: double (%f)
%	column16: double (%f)
%   column17: double (%f)
%	column18: double (%f)
%   column19: double (%f)
%	column20: double (%f)
%   column21: double (%f)
%	column22: double (%f)
%   column23: double (%f)
%	column24: double (%f)
%   column25: double (%f)
%	column26: double (%f)
%   column27: double (%f)
%	column28: double (%f)
%   column29: double (%f)
%	column30: double (%f)
%   column31: double (%f)
%	column32: double (%f)
%   column33: double (%f)
%	column34: double (%f)
%   column35: double (%f)
%	column36: double (%f)
%   column37: double (%f)
%	column38: double (%f)
%   column39: double (%f)
%	column40: double (%f)
%   column41: double (%f)
% For more information, see the TEXTSCAN documentation.
formatSpec = '%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%[^\n\r]';

%% Open the text file.
fileID = fopen(filename,'r');

%% Read columns of data according to the format.
% This call is based on the structure of the file used to generate this
% code. If an error occurs for a different file, try regenerating the code
% from the Import Tool.
dataArray = textscan(fileID, formatSpec, 'Delimiter', delimiter, 'TextType', 'string', 'HeaderLines' ,startRow-1, 'ReturnOnError', false, 'EndOfLine', '\r\n');

%% Close the text file.
fclose(fileID);

%% Post processing for unimportable data.
% No unimportable data rules were applied during the import, so no post
% processing code is included. To generate code which works for
% unimportable data, select unimportable cells in a file and regenerate the
% script.

%% Allocate imported array to column variable names
timestamp = dataArray{:, 1};
% ref_timestamp = dataArray{:, 2};
% ref_lat = dataArray{:, 3};
% ref_lon = dataArray{:, 4};
% surface_bottom_timestamp = dataArray{:, 5};
x = dataArray{:, 6};
y = dataArray{:, 7};
z = dataArray{:, 8};
% delta_xy0 = dataArray{:, 9};
% delta_xy1 = dataArray{:, 10};
% delta_z = dataArray{:, 11};
% vx = dataArray{:, 12};
% vy = dataArray{:, 13};
% vz = dataArray{:, 14};
% z_deriv = dataArray{:, 15};
% delta_vxy0 = dataArray{:, 16};
% delta_vxy1 = dataArray{:, 17};
% delta_vz = dataArray{:, 18};
% ax = dataArray{:, 19};
% ay = dataArray{:, 20};
% az = dataArray{:, 21};
% yaw = dataArray{:, 22};
% ref_alt = dataArray{:, 23};
% dist_bottom = dataArray{:, 24};
% dist_bottom_rate = dataArray{:, 25};
% eph = dataArray{:, 26};
% epv = dataArray{:, 27};
% evh = dataArray{:, 28};
% evv = dataArray{:, 29};
% estimator_type = dataArray{:, 30};
% xy_valid = dataArray{:, 31};
% z_valid = dataArray{:, 32};
% v_xy_valid = dataArray{:, 33};
% v_z_valid = dataArray{:, 34};
% xy_reset_counter = dataArray{:, 35};
% z_reset_counter = dataArray{:, 36};
% vxy_reset_counter = dataArray{:, 37};
% vz_reset_counter = dataArray{:, 38};
% xy_global = dataArray{:, 39};
% z_global = dataArray{:, 40};
% dist_bottom_valid = dataArray{:, 41};

timestart = timestamp(1);
for source_index = 1:length(timestamp)
    timestamp(source_index) = timestamp(source_index) - timestart;
    timestamp(source_index) = timestamp(source_index) * 1E-6;
end

% [timestamp_s,x_s,y_s,z_s,yaw_s,vx_s,vy_s,vz_s,acc_x,acc_y,acc_z] = importfile('/home/airlab/airlab_ws/Logs/log003_vehicle_local_position_setpoint_0.csv',2, 1285);
% timestart_s = timestamp_s(1);
% for source_index = 1:length(timestamp_s)
%     timestamp_s(source_index) = timestamp_s(source_index) - timestart_s;
%     timestamp_s(source_index) = timestamp_s(source_index) * 1E-6;
% end


z = -z;
z = medfilt1(z,10);
z = medfilt1(z,10);
% figure;
% subplot(3,1,1), plot(timestamp, x);
% subplot(3,1,2), plot(timestamp, y);
% subplot(3,1,3), plot(timestamp, z);

figure;
t_start = 1; % ~ 0s into flight
t_stop = length(timestamp); % log end
y_setpoint = 0.153; % y setpoint offset
y = y - y_setpoint;
x_setpoint = -0.2535;
x = x - x_setpoint;

pos_start = 731; %log start at 41:28.5, pos hold at 42:42.5
pos_end = 3449; %pos end at 47:19.5 i.e. 4 mins 37 s after pos_start
% x(1:pos_start) = 0; 
% y(1:pos_start) = 0;
% 
% x(pos_end:end) = 0;
% y(pos_end:end) = 0;

% generate for tunnel boundaries
R = 2.5;
depth = -4;
[X, Y, Z]      = cylinder(R,100);
Z = (Z*depth);
% Z           = meshgrid(0::2)
CO(:,:,1)   = zeros(size(Z));
CO(:,:,2)   = zeros(size(Z));
CO(:,:,3)   = zeros(size(Z));

% plot cylinder tunnel wall

% left tunnel wall
surf(X,Y,Z,CO,'EdgeColor','none','FaceAlpha',0.35);
hold on;

% top tunnel wall
% [X1, Y1]      = meshgrid(-2.5:1:2.5, -2.5:1:2.5);
% Z1           = 2.0 * ones(size(X1));
% CO1(:,:,1)   = zeros(size(Z1));
% CO1(:,:,2)   = zeros(size(Z1));
% CO1(:,:,3)   = zeros(size(Z1));
% surf(X1,Y1,Z1,'EdgeColor','none','FaceAlpha',0.1);

% bottom tunnel wall
% Z1          = 0 * ones(size(X1));
% CO1(:,:,1)   = (51/255)*ones(size(Z1));
% CO1(:,:,2)   = (153/255)*ones(size(Z1));
% CO1(:,:,3)   = (255/255)*ones(size(Z1));
% surf(X1,Y1,Z1,CO1,'EdgeColor','none','FaceAlpha',1);
xlabel('x (m)')
ylabel('y (m)')
zlabel('altitude (m)')
% hold off

%% 3d tunnel animation
% plot3(x(t_start:t_stop), y(t_start:t_stop), z(t_start:t_stop), '-', 'Color', 'k');
p = plot3(x(t_start), y(t_start), z(t_start), 'Marker', 'o','MarkerFaceColor','red', 'MarkerEdgeColor','red', 'MarkerSize', 12);
py = plot3(-R, y(t_start), depth, 'Marker', 'o','MarkerFaceColor',[0.5 0.5 0.5], 'MarkerEdgeColor','[0.5 0.5 0.5]', 'MarkerSize', 6);
px = plot3(x(t_start), R, depth, 'Marker', 'o','MarkerFaceColor',[0.5 0.5 0.5], 'MarkerEdgeColor','[0.5 0.5 0.5]', 'MarkerSize', 6);

% 2d cross-section animation
% % plot3(y(t_start:t_stop), linspace(0,(t_stop-t_start)/10, length(y(t_start:t_stop))), z(t_start:t_stop), '-', 'Color', 'k');
% p = plot3(y(t_start), 0, z(t_start),'o','MarkerFaceColor','red');
% view([0,0])

history_h1 = animatedline;
history_h1.LineWidth = 4;
history_h1.MaximumNumPoints = 1;
history_h1.Color = 'yellow';

% history_h2 = animatedline;
% history_h2.LineWidth = 2;
% history_bool = true;
% history_count = 0;
% history_n = 20;

xlim([-R R])
% ylim([0 30])
ylim([-R R])
zlim([0 5])
axis equal
el = 90;
% az = -51 - 90;
az = 0;

% view([az, el]);
animate = true;
view_3d = false;
view_2d = ~view_3d;

if (animate && view_3d)
    pz = plot3(-R, -R, z(t_start), 'Marker', 'o','MarkerFaceColor',[0.5 0.5 0.5], 'MarkerEdgeColor','[0.5 0.5 0.5]', 'MarkerSize', 6);
    set(gca, 'YDir', 'reverse')    
    v = VideoWriter('output_offset_3d.avi', 'Uncompressed AVI');
    v.FrameRate = 10;
    
    open(v);
    
    for i=t_start:t_stop
        if (z(i) > 0)
            z(i) = 0;
        end
        % add trail
        addpoints(history_h1, x(i), y(i), z(i));
        %     addpoints(history_h1, x(i), y(i), 0);
        
        % LOL can just use MaximumNumPoints instead
        %     if (history_bool)
        %         history_count = history_count + 1;
        %         addpoints(history_h1, x(i), y(i), z(i));
        %         if (history_count >= history_n)
        %             history_count = 0;
        %             clearpoints(history_h1);
        %             for j = 0:history_n-1
        %                 history_count = history_count + 1;
        %                 addpoints(history_h2, x(i-j), y(i-j), z(i-j));
        %             end
        %
        %             history_bool = ~history_bool;
        %         end
        %     else
        %         history_count = history_count + 1;
        %         addpoints(history_h2, x(i), y(i), z(i));
        %         if (history_count >= history_n)
        %             history_count = 0;
        %             clearpoints(history_h2);
        %             for j = 0:history_n-2
        %                 history_count = history_count + 1;
        %                 addpoints(history_h1, x(i-j), y(i-j), z(i-j));
        %             end
        %
        %             history_bool = ~history_bool;
        %         end
        %     end
        
        
        %     if (history_bool)
        %         if (i>1)
        %             addpoints(history_h1, x(i-1), y(i-1), z(i-1));
        %         end
        %     	addpoints(history_h2, x(i), y(i), z(i));
        %     else
        %         if (i>1)
        %             addpoints(history_h1, x(i-1), y(i-1), z(i-1));
        %         end
        %     end
        %
        %     history_count = history_count + 1;
        %
        %     % remove trail
        %
        %     if (history_count > 15)
        %         if (history_bool)
        %             clearpoints(history_h1);
        %         else
        %             clearpoints(history_h2);
        %         end
        %         history_bool = ~history_bool;
        %         history_count = 0;
        %     end
        if (i >= pos_start && i<= pos_end)
            p.MarkerFaceColor = [0 1 0];
            p.MarkerEdgeColor = [0 1 0];
            history_h1.MaximumNumPoints = 50;
        else
            p.MarkerFaceColor = 'red';
            p.MarkerEdgeColor = 'red';
            history_h1.MaximumNumPoints = 1;
        end
        
        p.XData = x(i);
        p.YData = y(i);
        p.ZData = z(i);
        
        py.XData = -R; py.YData = y(i); py.ZData = depth;
        px.XData = x(i); px.YData = R; px.ZData = depth;
        pz.XData = -R; pz.YData = -R; pz.ZData = z(i);
        
        
%         pause(0.1)
        drawnow
        frame = getframe(gcf);
        writeVideo(v, frame);
        axis equal
        % % hold on;
        % % plot3(y(i), timestamp(i), z(i), '.');
        
        % % plot3(y(i),(i-t_start)/10,z(i),'o','MarkerFaceColor','red');
        % % for j=t_start:i-1
        % %     plot3(y(j), (j-t_start)/10, z(j), '.', 'Color', [1 1 1]);
        % % end
        % xlim([-3 3])
        % ylim([0 (t_stop-t_start)/10])
        % zlim([0 2])
        % axis equal
        % % hold on
        % pause(0.1)
        % % hold off
    end
    
    close(v);
    
    
end

if (animate && view_2d)
    az = 90; el = 90;
    view([az, el]);
    set(gca, 'XDir', 'reverse')   
%     set(gca, 'YDir', 'reverse')   
    
    v = VideoWriter('output_offset_2d.avi', 'Uncompressed AVI');
    v.FrameRate = 10;
    
    open(v);
    
    for i=t_start:t_stop
        if (z(i) > 0)
            z(i) = 0;
        end
        % add trail
        addpoints(history_h1, x(i), y(i), z(i));
        %     addpoints(history_h1, x(i), y(i), 0);
        
        % LOL can just use MaximumNumPoints instead
        %     if (history_bool)
        %         history_count = history_count + 1;
        %         addpoints(history_h1, x(i), y(i), z(i));
        %         if (history_count >= history_n)
        %             history_count = 0;
        %             clearpoints(history_h1);
        %             for j = 0:history_n-1
        %                 history_count = history_count + 1;
        %                 addpoints(history_h2, x(i-j), y(i-j), z(i-j));
        %             end
        %
        %             history_bool = ~history_bool;
        %         end
        %     else
        %         history_count = history_count + 1;
        %         addpoints(history_h2, x(i), y(i), z(i));
        %         if (history_count >= history_n)
        %             history_count = 0;
        %             clearpoints(history_h2);
        %             for j = 0:history_n-2
        %                 history_count = history_count + 1;
        %                 addpoints(history_h1, x(i-j), y(i-j), z(i-j));
        %             end
        %
        %             history_bool = ~history_bool;
        %         end
        %     end
        
        
        %     if (history_bool)
        %         if (i>1)
        %             addpoints(history_h1, x(i-1), y(i-1), z(i-1));
        %         end
        %     	addpoints(history_h2, x(i), y(i), z(i));
        %     else
        %         if (i>1)
        %             addpoints(history_h1, x(i-1), y(i-1), z(i-1));
        %         end
        %     end
        %
        %     history_count = history_count + 1;
        %
        %     % remove trail
        %
        %     if (history_count > 15)
        %         if (history_bool)
        %             clearpoints(history_h1);
        %         else
        %             clearpoints(history_h2);
        %         end
        %         history_bool = ~history_bool;
        %         history_count = 0;
        %     end
        if (i >= pos_start && i<= pos_end)
            p.MarkerFaceColor = [0 1 0];
            p.MarkerEdgeColor = [0 1 0];
            history_h1.MaximumNumPoints = 50;
        else
            p.MarkerFaceColor = 'red';
            p.MarkerEdgeColor = 'red';
            history_h1.MaximumNumPoints = 1;
        end
        
        p.XData = x(i);
        p.YData = y(i);
        p.ZData = z(i);
        
        py.XData = -R; py.YData = y(i); py.ZData = depth;
        px.XData = x(i); px.YData = -R; px.ZData = depth;
%         pz.XData = -R; pz.YData = -R; pz.ZData = z(i);
        
        
%         pause(0.1)
        drawnow
        frame = getframe(gcf);
        writeVideo(v, frame);
        axis equal
        % % hold on;
        % % plot3(y(i), timestamp(i), z(i), '.');
        
        % % plot3(y(i),(i-t_start)/10,z(i),'o','MarkerFaceColor','red');
        % % for j=t_start:i-1
        % %     plot3(y(j), (j-t_start)/10, z(j), '.', 'Color', [1 1 1]);
        % % end
        % xlim([-3 3])
        % ylim([0 (t_stop-t_start)/10])
        % zlim([0 2])
        % axis equal
        % % hold on
        % pause(0.1)
        % % hold off
    end
    
    close(v);
    
    
end



% h_2d = figure();
% for i=t_start:t_stop
%     plot(y(i), z(i), 'o','MarkerFaceColor','red');
%     hold on;
%     plot(linspace(-3,3,100), linspace(0,2,100))
%     xlim([-3 3])
%     % ylim([0 30])
%     ylim([0 2])
%     axis equal
%     pause(0.1)
% end



% for i=t_start:t_stop
%     h = animatedline(y(t_start:i), 0:0.1:((i-t_start)/10) ,z(t_start:i))
%     clearpoints(h)
%     pause(0.1)
%     h = animatedline(y(i+1), (i-t_start+1)/10 ,z(i+1),'Color', 'red', 'Marker', '.', 'MaximumNumPoints', 5)
%     addpoints(h, y(i), (i-t_start)/10, z(i));
%     drawnow
%     plot3(y(i), (i-t_start)/10, z(i), '-', 'Color', 'k');
%    pause(0.1)
% end

%% Calculate Errors

errors = false;

if (errors)
    
    % RMS
x_abs = abs(x(pos_start:pos_end));;
x_rms = rms(x(pos_start:pos_end))
x_max = max(abs(x(pos_start:pos_end)))

y_abs = abs(y(pos_start:pos_end));
y_rms = rms(y(pos_start:pos_end))
y_max = max(abs(y(pos_start:pos_end)))

% Euclidean Error
r = sqrt(x(pos_start:pos_end).^2 + y(pos_start:pos_end).^2);
r_abs = abs(r);
r_max = max(r)
r_rms = rms(r)

% absolute error vs distance travelled plot
z_v = diff(z(pos_start:pos_end));
z_r = cumtrapz(z_v);
z_distance = cumtrapz(abs(z_v));
total_distance = z_distance(end)
total_time = timestamp(pos_end) - timestamp(pos_start)

figure()
plot(z_distance, r(2:end), 'LineWidth', 2)
hold on
plot(z_distance, r_max*ones(size(z_distance)), 'LineWidth', 2)
plot(z_distance, r_rms*ones(size(z_distance)), 'LineWidth', 2)
hold off
xlabel('Distance Travelled (m)')
ylabel('Absolute Error (m)')
legend('Abs Error', 'Max Error', 'RMS Error')
t1 = text(z_distance(end), r_rms, [' ' num2str(round(r_rms,2))]);
t2 = text(z_distance(end), r_max, [' ' num2str(round(r_max,2))]);
% t1.HorizontalAlignment = 'right'
ylim([0 0.4])
xlim([0 z_distance(end)])

figure()
hold on
plot(z_distance, x_abs(2:end), 'r-', 'LineWidth', 0.1)
plot(z_distance, y_abs(2:end), 'b-', 'LineWidth', 0.1)
plot(z_distance, r(2:end), 'k-', 'LineWidth', 3)
hold off
xlabel('Distance Travelled (m)')
ylabel('Absolute Errors (m)')
legend('Abs x Error', 'Abs y Error', 'Abs r Error')
% t1.HorizontalAlignment = 'right'
ylim([0 0.4])
xlim([0 z_distance(end)])

figure()

subplot(2,1,1)
hold on
plot(z_distance, x(pos_start:pos_end-1), 'r-', 'LineWidth', 1)
plot(z_distance, y(pos_start:pos_end-1), 'b-', 'LineWidth', 1)

% plot(z_distance, r(2:end), 'k-', 'LineWidth', 3)
xlabel('Distance Travelled (m)')
ylabel('Position (m)')
legend('x Position', 'y Position')
% t1.HorizontalAlignment = 'right'
ylim([-0.4 0.4])
xlim([0 z_distance(end)])

subplot(2,1,2)
hold on
plot(z_distance, x_abs(1:end-1), 'r-', 'LineWidth', 1)
plot(z_distance, y_abs(1:end-1), 'b-', 'LineWidth', 1)
% plot(z_distance, r(2:end), 'k-', 'LineWidth', 3)
hold off
xlabel('Distance Travelled (m)')
ylabel('Absolute Error (m)')
legend('Abs x Error', 'Abs y Error')
% t1.HorizontalAlignment = 'right'
ylim([0 0.4])
xlim([0 z_distance(end)])

% figure()
% % plot(z(pos_start:pos_end)); 
% hold on;
% plot(z_r, '--');plot(z_v, '.-');plot(z_distance);

    
end


%% Clear temporary variables
clearvars filename delimiter startRow formatSpec fileID dataArray ans;