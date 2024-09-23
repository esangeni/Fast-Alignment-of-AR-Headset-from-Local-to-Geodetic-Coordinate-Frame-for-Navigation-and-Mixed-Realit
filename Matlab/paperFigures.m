clear all; clc; close all; warning('off')
set(0,'DefaultTextInterpreter', 'latex');
set(0,'DefaultAxesTickLabelInterpreter','latex');
set(0,'DefaultLegendInterpreter','latex');

%% Configuration Parameters
% Engineering Gateway outdoor data:
init_latitude = 33.64321245745905;
init_longitude = -117.84020287319062;
init_altitude = 0;
goal_latitude = 33.64349188103053; % goal_latitude = 33.64348782340351; 
goal_longitude = -117.84046049876312; % goal_longitude = -117.84045767509454;
goal_altitude = 0;

load('resultsML2.mat')
load('resultsICM.mat')

%% ------ Libraries ------
currDir = pwd;

addpath([currDir, '\lib\GoogleMapPlot\']);

%% --------------------
%  ENU Plot
% --------------------
wgs84 = wgs84Ellipsoid('meter');
ecef_goal = lla2ecef([goal_latitude, goal_longitude, goal_altitude]);
ecef_init = lla2ecef([init_latitude, init_longitude, init_altitude]);
[xEast_goal, yNorth_goal, zUp_goal] = ecef2enu(ecef_goal(1), ecef_goal(2), ecef_goal(3), init_latitude, init_longitude, init_altitude, wgs84);
[xEast_init, yNorth_init, zUp_init] = ecef2enu(ecef_init(1), ecef_init(2), ecef_init(3), init_latitude, init_longitude, init_altitude, wgs84);

figure('Name','ENU Frame - ENU Mag vs Visual Aligment'), hold on,
    title('Trajectories - Navigation Frame')
    legend('ENU ML2', 'ENU ML2 Alig', 'ENU Matlab')
    xlabel('East [m]'), ylabel('North [m]'), zlabel('Up [m]');
    j = 4;
    for i = 1000:100:1900
        fieldName = sprintf('exp%d_ENU', i);
        enu_plot(j) = plot3(resultsICM.(fieldName)(1,:), resultsICM.(fieldName)(2,:), resultsICM.(fieldName)(3,:),'Color',"#A2142F"); hold on;
        j = j + 1;
        enu_plot(j) = plot3(resultsML2.(fieldName)(1,:), resultsML2.(fieldName)(2,:), resultsML2.(fieldName)(3,:),'Color',"#00BFFF"); hold on;
        j = j + 1;
        enu_plot(j) = plot(resultsICM.(fieldName)(1, end), resultsICM.(fieldName)(2, end), '*', 'Color',"#A2142F", 'MarkerSize', 4); hold on
        j = j + 1;
        enu_plot(j) = plot(resultsML2.(fieldName)(1, end), resultsML2.(fieldName)(2, end), '*', 'Color',"#00BFFF", 'MarkerSize', 4); hold on
        j = j + 1;
    end
    enu_plot(1) = plot3(xEast_goal, yNorth_goal, zUp_goal, 'sk','MarkerFaceColor','k', 'MarkerSize', 5); hold on;
    enu_plot(2) = plot3(xEast_init, yNorth_init, zUp_init, 'ok', 'MarkerSize', 4); hold on;
    enu_plot(3) = plot3([xEast_init, xEast_goal], [yNorth_init, yNorth_goal], [zUp_init, zUp_goal], 'k-','LineWidth',2); hold on;
    grid on;
    legend([enu_plot(1) enu_plot(2) enu_plot(3) enu_plot(4) enu_plot(5)],'$LLA_{Goal}$', '$LLA_{Initial}$', '$LLA_{True}$', '$LLA_{MN}$', '$LLA_{AR_{Visual}}$');

%% --------------------
%  Stadistics
% --------------------
% 1. Compute True Bearing with init (lat,lon) and goal (lat,lon)
    % Convert latitude and longitude from degrees to radians
    init_lat_rad = deg2rad(init_latitude);
    init_lon_rad = deg2rad(init_longitude);
    goal_lat_rad = deg2rad(goal_latitude);
    goal_lon_rad = deg2rad(goal_longitude);
    
    % Compute the difference in longitude
    delta_lon = goal_lon_rad - init_lon_rad;
    
    % Compute the bearing
    y = sin(delta_lon) * cos(goal_lat_rad);
    x = cos(init_lat_rad) * sin(goal_lat_rad) - sin(init_lat_rad) * cos(goal_lat_rad) * cos(delta_lon);
    bearing_rad = atan2(y, x);

    % Convert the bearing from radians to degrees
    true_bearing_deg = rad2deg(bearing_rad);
    
    % Normalize the bearing to the range [0, 360)
    true_bearing_deg = mod(true_bearing_deg + 360, 360);
    
    % Display the result
    disp(['True Bearing from (initial, goal):', num2str(true_bearing_deg),' [deg]']);

     
% 2. Compute Bearings MN
    bearingsMN_TN = zeros(1,10); j=1;
    for i = 1000:100:1900
        fieldName = sprintf('exp%d_TN', i);
        bearingsMN_TN(1,j) = resultsICM.(fieldName);
        j = j+1;
    end

    disp('Errors for MN Approx with compensation to TN:')
    disp('---------------------------------------------')
    disp("MN Bearings to TN: {" + num2str(bearingsMN_TN) + "}")
    % Compute the Mean Angular Error:
    % The mean angular error is the average of the absolute differences
    % between the true bearing and the approximated bearings.
    % This metric is simple and effective for angular data.
    errors = abs(bearingsMN_TN - true_bearing_deg);
    mean_angular_error = mean(errors);
    disp(['Mean Angular Error: ', num2str(mean_angular_error), ' [deg]']);

    % Root Mean Square Error (RMSE):
    errors_squared = (bearingsMN_TN - true_bearing_deg).^2;
    rmse = sqrt(mean(errors_squared));
    
    disp(['Root Mean Square Error: ', num2str(rmse), ' [deg]']);

% 3. Compute Bearings ARVisual
    finishLLAML2_deg = zeros(2,10);
    finishLLAML2_rad = zeros(2,10);
    offsetsVisual = zeros(1,10); j=1;
    for i = 1000:100:1900
        fieldName = sprintf('exp%d_TN', i);
        offsetsVisual(1,j) = resultsML2.(fieldName);
        fieldName = sprintf('exp%d_LLA', i);
        finishLLAML2_deg(1,j) = resultsML2.(fieldName)(end,1); % Latitude
        finishLLAML2_deg(2,j) = resultsML2.(fieldName)(end,2); % Longitude
        j = j+1;
    end

    finishLLAML2_rad = deg2rad(finishLLAML2_deg);
    bearingML2_deg = zeros(1,length(finishLLAML2_rad));

    for i = 1:length(finishLLAML2_rad)
        % Compute the difference in longitude
        delta_lon = finishLLAML2_rad(2,i) - init_lon_rad;
        
        % Compute the bearing
        y = sin(delta_lon) * cos(finishLLAML2_rad(1,i));
        x = cos(init_lat_rad) * sin(finishLLAML2_rad(1,i)) - sin(init_lat_rad) * cos(finishLLAML2_rad(1,i)) * cos(delta_lon);
        bearing_rad = atan2(y, x);
    
        % Convert the bearing from radians to degrees
        bearing_deg = rad2deg(bearing_rad);
        
        % Normalize the bearing to the range [0, 360)
        bearingML2_deg(1,i) = mod(bearing_deg + 360, 360);
    end

    disp(['---------------------------------------------']);
    disp(['Errors for ML with AR Visual Alignment:']);
    disp(['---------------------------------------------']);
    disp("AR Bearings to TN: {" + num2str(bearingML2_deg) + "}")
    % Compute the Mean Angular Error:
    % The mean angular error is the average of the absolute differences
    % between the true bearing and the approximated bearings.
    % This metric is simple and effective for angular data.
    errors = abs(bearingML2_deg - true_bearing_deg);
    mean_angular_error = mean(errors);
    disp(['Mean Angular Error: ', num2str(mean_angular_error), ' [deg]']);

    % Root Mean Square Error (RMSE):
    errors_squared = (bearingML2_deg - true_bearing_deg).^2;
    rmse = sqrt(mean(errors_squared));
    
    disp(['Root Mean Square Error: ', num2str(rmse), ' [deg]']);

% 4. Mean Percentage Error respect to True Bearing
% 5. Compute Standard Deviations respect to True Bearing for MN and
% ARVisual

% 6. CEPs:
    goalENU =  [xEast_goal; yNorth_goal];
    finishENU_ML2 = zeros(2,10);
    finishENU_ICM = zeros(2,10);
    
    j = 1;
    for i = 1000:100:1900
        fieldName = sprintf('exp%d_ENU', i);
        finishENU_ML2(:,j) = resultsML2.(fieldName)(1:2, end);
        finishENU_ICM(:,j) = resultsICM.(fieldName)(1:2, end);
        j = j+1;
    end
    
    % Compute the Euclidean distances from the true position
    distancesML2 = sqrt(sum((finishENU_ML2' - goalENU').^2, 2));
    distancesICM = sqrt(sum((finishENU_ICM' - goalENU').^2, 2));
    
    % Sort the distances
    sorted_distances_ML2 = sort(distancesML2);
    sorted_distances_ICM = sort(distancesICM);
    
    % Find the distance that corresponds to the 50th percentile (median)
    CEP_index = ceil(10 / 2);
    CEP_ML2 = sorted_distances_ML2(CEP_index);
    CEP_ICM = sorted_distances_ICM(CEP_index);
    
    % Display the result
    disp(['Circular Probable Error (CEP) ICM trajectories: ', num2str(CEP_ICM), ' [m]']);
    disp(['Circular Probable Error (CEP) ML2 trajectories: ', num2str(CEP_ML2), ' [m]']);
    
    figure('Name','CEPS: ENU Frame - ENU Mag vs Visual Aligment'), hold on,
        title('Circular Error Probable - Navigation Frame')
        legend('ENU ML2', 'ENU ML2 Alig', 'ENU Matlab')
        xlabel('East [m]'), ylabel('North [m]'), zlabel('Up [m]');
        
        enu_plot(1) = viscircles(goalENU', CEP_ICM, 'Color',"#A2142F",'LineStyle', '--');
        enu_plot(2) = viscircles(goalENU', CEP_ML2, 'Color',"#00BFFF",'LineStyle', '--');
        j = 3;
        for i = 1000:100:1900
            fieldName = sprintf('exp%d_ENU', i);
            enu_plot(j) = plot(resultsICM.(fieldName)(1, end), resultsICM.(fieldName)(2, end), '^','Color',"#A2142F",'LineWidth',2, 'MarkerSize', 6); hold on
            j = j + 1;
            enu_plot(j) = plot(resultsML2.(fieldName)(1, end), resultsML2.(fieldName)(2, end), '^','Color',"#00BFFF",'LineWidth',2, 'MarkerSize', 6); hold on
            j = j + 1;
        end
        enu_plot(j) = plot(xEast_goal, yNorth_goal, 'o','Color',"k",'LineWidth',3, 'MarkerSize', 6); hold on;
        grid on; axis equal;
        legend([enu_plot(j) enu_plot(3) enu_plot(1) enu_plot(4) enu_plot(2)],'$True_{Destination}$', '$MN$', '$CEP (r=0.82 [m])$', '$AR_{Visual}$', '$CEP (r=0.28 [m])$','Location','southeast');

% 7. Compute the Standard deviation of the Angular Error:
% Errors
errors_MN = abs(bearingsMN_TN - true_bearing_deg);
errors_AR = abs(bearingML2_deg - true_bearing_deg);

% Mean Angular Errors (already provided)
mean_error_MN = mean(errors_MN);
mean_error_AR = mean(errors_AR);

% Standard Deviations
std_dev_MN = std(errors_MN);
std_dev_AR = std(errors_AR);

% Display the results
fprintf('Standard Deviation of Angular Errors for MN: %.3f [deg]\n', std_dev_MN);
fprintf('Standard Deviation of Angular Errors for AR Visual: %.3f [deg]\n', std_dev_AR);


%% --------------------
%  Google Maps Plot
% --------------------
figure('Name','Google Maps'), hold on
    xlabel('Longitude ($^{\circ}$)', 'Interpreter', 'latex')
    ylabel('Latitude ($^{\circ}$)', 'Interpreter', 'latex')
    axis equal
    title('Google Maps Trajectory', 'Interpreter', 'latex');
    xlim([init_longitude-0.00010 init_longitude+0.00010])
    ylim([init_latitude-0.0001 init_latitude+0.0004])
    set(gca, 'TickLabelInterpreter', 'latex');
    set(gca,'DefaultTextInterpreter', 'latex');
    set(gca,'DefaultAxesTickLabelInterpreter','latex');
    set(gca,'DefaultLegendInterpreter','latex');
    
    map_plot(1) = plot_google_map('MapType','hybrid', 'MapScale', 1, 'Scale', 2, 'ShowLabels', 0); hold on; j = 7;
    for i = 1000:100:1900
        fieldName = sprintf('exp%d_LLA', i);
        map_plot(5) = plot(resultsICM.(fieldName)(:,2), resultsICM.(fieldName)(:,1), 'Color', "#A2142F", 'LineWidth', 1); hold on; % Bright Orange Color
        map_plot(6) = plot(resultsML2.(fieldName)(:,2), resultsML2.(fieldName)(:,1), 'Color', "#00BFFF", 'LineWidth', 1); hold on; % Bright Sky Blue Color
        map_plot(j) = plot(resultsICM.(fieldName)(end,2), resultsICM.(fieldName)(end,1), '*', 'Color', "#A2142F", 'MarkerSize', 8); hold on; % Bright Orange Color
        j = j + 1;   
        map_plot(j) = plot(resultsML2.(fieldName)(end,2), resultsML2.(fieldName)(end,1), '*', 'Color', "#00BFFF", 'MarkerSize', 8); hold on; % Bright Sky Blue Color
        j = j + 1;
    end
    map_plot(2) = plot(goal_longitude, goal_latitude, 'sk', 'MarkerFaceColor', 'k', 'MarkerSize', 6, 'MarkerFaceColor', 'k'); hold on;  % Black Color
    map_plot(3) = plot([init_longitude, goal_longitude], [init_latitude, goal_latitude], 'k-', 'LineWidth', 2); hold on; % Black Color
    map_plot(4) = plot(init_longitude, init_latitude, 'ok', 'MarkerSize', 4, 'MarkerFaceColor', 'k'); hold on; % Black Color
    legend([map_plot(2) map_plot(4) map_plot(3) map_plot(5) map_plot(6)], '$LLA_{Goal}$', '$LLA_{Initial}$', '$LLA_{True}$', '$LLA_{MN}$', '$LLA_{AR_{Visual}}$');
