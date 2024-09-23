clear all; clc; close all;
set(0,'DefaultTextInterpreter', 'latex');
set(0,'DefaultAxesTickLabelInterpreter','latex');
set(0,'DefaultLegendInterpreter','latex');

%% ------ Libraries ------
currDir = pwd;
addpath([currDir, '\lib\GoogleMapPlot\']);

%% Configuration Parameters
ifXYZPlot = false;
ifXYZvsAlignedPlot = false;
ifENUvsAlignedPlot = false;
ifLLAvsAlignedPlot = false;

% Engineering Gateway outdoor data:
init_latitude = 33.64321245745905;
init_longitude = -117.84020287319062;
init_altitude = 0;
goal_latitude = 33.64348782340351;
goal_longitude = -117.84045767509454;
goal_altitude = 0;


figure('Name','Google Maps'), hold on
xlabel('Longitude ($^{\circ}$)'), ylabel('Latitude ($^{\circ}$)'), axis equal
title('Google Maps Trajectory');
xlim([init_longitude-0.00015 init_longitude+0.00015]), ylim([init_latitude-0.0003 init_latitude+0.0003])
set(gca, 'TickLabelInterpreter', 'latex');
set(gca,'DefaultTextInterpreter', 'latex');
set(gca,'DefaultAxesTickLabelInterpreter','latex');
set(gca,'DefaultLegendInterpreter','latex');
%% Results:
for i = 1000:100:1900

    ml2DataFile = ['Datasets/7_19_2024/exp', num2str(i)];
    
    % Reading the stored data and importing it into a table
    opts = delimitedTextImportOptions('Delimiter', ',');
    opts.VariableNames = {'Timestamp', 'AligmentOffset',...
                          'X', 'Y', 'Z', ...
                          'X_Align', 'Y_Align', 'Z_Align',...
                          'East', 'North', 'Up',...
                          'East_Align', 'North_Align', 'Up_Align',...
                          'X_ECEF', 'Y_ECEF', 'Z_ECEF', ...
                          'X_ECEFAlign', 'Y_ECEFAlign', 'Z_ECEFAlign', ...
                          'Lat', 'Lon', 'Alt', ...
                          'Lat_Align', 'Lon_Align', 'Alt_Align'};
    
    opts.VariableTypes = {'double', 'double', 'double', 'double', 'double', ...
                          'double', 'double', 'double', 'double', 'double', ...
                          'double', 'double', 'double', 'double', 'double', ...
                          'double', 'double', 'double', 'double', 'double', ...
                          'double', 'double', 'double', 'double', 'double','double'};
    
    
    ml2Data = readtable(ml2DataFile, opts);
    
    % Display head of imported table for verification
    ml2Data(1, :) = []; % Remove initial row which is 0,0,0...
    disp('Table after removing the first row:')
    disp(head(ml2Data))
    
    % Left Handed XYZ Unity World Coordinate Frame (UWC-Frame):
    UWC_left = [ml2Data.X, ml2Data.Y, ml2Data.Z]';
    
    offset_Deg = -ml2Data.AligmentOffset(1,1)-2.5; % -2.5 [deg] Visual Misaligment of ML2
    
    % Unity Aligment to Target
    R_y_Offset = [cosd(offset_Deg) 0 sind(offset_Deg);...
                                0  1                0;...
                 -sind(offset_Deg) 0 cosd(offset_Deg)];

    UWC_left = R_y_Offset*UWC_left;
    
    % x: axis points to the right.
    % y: axis points to the Up.
    % z: axis points to the forward.
    UWC_right = [1 0  0;...
                 0 1  0;...
                 0 0 -1] * UWC_left;
    
    % Unity to ENU
    % x: axis points to the East.
    % y: axis points to the North.
    % z: axis points to the Up.
    ENU = [-1 0  0;...
            0 0 1;...
            0 -1  0] * UWC_right;
    
    wgs84 = wgs84Ellipsoid('meter');
    [X,Y,Z] = enu2ecef(ENU(1,:), ENU(2,:), ENU(3,:), init_latitude, init_longitude, init_altitude, wgs84);
    ECEF = [X', Y', Z'];
    lla = ecef2lla(ECEF, 'WGS84');

    % --------------------
    % Save Data
    % --------------------
    % Construct the field name
    fieldName = sprintf('exp%d_UWCLeft', i);
    resultsML2.(fieldName) = UWC_left;
    fieldName = sprintf('exp%d_UWCRight', i);
    resultsML2.(fieldName) = UWC_right;
    fieldName = sprintf('exp%d_ENU', i);
    resultsML2.(fieldName) = ENU;
    fieldName = sprintf('exp%d_LLA', i);
    resultsML2.(fieldName) = lla;
    fieldName = sprintf('exp%d_TN', i);
    resultsML2.(fieldName) = mod(offset_Deg,360);
    
    % --------------------
    %  Google Maps Plot
    % --------------------
    map_plot(1) = plot_google_map('MapType','hybrid','MapScale', 1,'Scale',1.5,'ShowLabels',0); hold on
    map_plot(2) = plot(goal_longitude, goal_latitude, 'sk', 'MarkerSize', 4, 'MarkerFaceColor', 'k');
    map_plot(3) = plot(ml2Data.Lon, ml2Data.Lat, 'b', 'LineWidth', 1);
    map_plot(4) = plot(lla(:,2), lla(:,1), 'r', 'LineWidth', 1);
    legend([map_plot(2) map_plot(3) map_plot(4)],'$LLA_{Goal}$', '$LLA_{\overline{Aligned}}$', '$LLA_{Aligned}$', 'Interpreter', 'latex');
end

save("resultsML2.mat","resultsML2")