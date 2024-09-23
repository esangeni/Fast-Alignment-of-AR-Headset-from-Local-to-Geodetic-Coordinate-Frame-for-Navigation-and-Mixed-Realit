clear all; clc; close all;
set(0,'DefaultTextInterpreter', 'latex');
set(0,'DefaultAxesTickLabelInterpreter','latex');
set(0,'DefaultLegendInterpreter','latex');

%% Configuration Parameters
% paper experiments: [exp1000...to...exp1900]
exp = 1900;
ml2DataFile = ['Datasets/7_19_2024/exp', num2str(exp), '_dataICM20948'];

% Engineering Gateway outdoor data:
init_latitude = 33.64321245745905;
init_longitude = -117.84020287319062;
init_altitude = 0;
goal_latitude = 33.64348782340351;
goal_longitude = -117.84045767509454;
goal_altitude = 0;

%% ------ Libraries ------
currDir = pwd;
addpath([currDir, '\lib\GoogleMapPlot\']);

%% ------ ML2 Data Process ------

% Reading the stored data and importing it into a table
opts = delimitedTextImportOptions('Delimiter', ',');
opts.VariableNames = {'Time', 'Temp', ... % [s], [degC]
                      'AccX', 'AccY', 'AccZ', ... % [m/s^2]
                      'GyrX', 'GyrY', 'GyrZ',... % [rad/s]
                      'MagX', 'MagY', 'MagZ'}; % [uT]

opts.VariableTypes = {'double', 'double', ...
                      'double', 'double', 'double', ...
                      'double', 'double', 'double', ...
                      'double', 'double', 'double'};


icmData = readtable(ml2DataFile, opts);

% Display head of imported table for verification
disp('Table after removing the first row:')
disp(head(icmData))

figure('Name','IMU readings:'),
    subplot(3,1,1), plot(icmData.Time, icmData.AccX, icmData.Time, icmData.AccY, icmData.Time, icmData.AccZ)
    title('Accelerometer Data')
    subplot(3,1,2), plot(icmData.Time, icmData.GyrX, icmData.Time, icmData.GyrY, icmData.Time, icmData.GyrZ)
    title('Gyroscope Data')
    subplot(3,1,3), plot(icmData.Time, icmData.MagX, icmData.Time, icmData.MagY, icmData.Time, icmData.MagZ)
    title('Magnetometer Data')

%% Aligment Readings between sensors:
% Aligment Acc and Gyr to Magnetometer Frame:
Acc = [icmData.AccX, icmData.AccY, icmData.AccZ]';
Gyr = [icmData.GyrX, icmData.GyrY, icmData.GyrZ]';
R_AccGyr_Mag = [1 0 0; 0 -1 0; 0 0 -1];

Acc_rot = R_AccGyr_Mag*Acc;
Gyr_rot = R_AccGyr_Mag*Gyr;

icmData.AccXRot = Acc_rot(1,:)';
icmData.AccYRot = Acc_rot(2,:)';
icmData.AccZRot = Acc_rot(3,:)';

icmData.GyrXRot = Gyr_rot(1,:)';
icmData.GyrYRot = Gyr_rot(2,:)';
icmData.GyrZRot = Gyr_rot(3,:)';

%% Magnetometer Calibration
calTime = [41, 28, 22, 24, 43.1, 52.1, 27, 38, 32, 37.7, 29.3];

calTime_ = calTime((exp-1000)/100+1);

Mag = [icmData.MagX, icmData.MagY, icmData.MagZ]';
icmCalData = icmData(icmData.Time <= calTime_, :);
calMagData = [icmCalData.MagX, icmCalData.MagY, icmCalData.MagZ]';

[A,b,expmfs] = magcal(calMagData'); % Calibration coefficients
expmfs % Display the expected magnetic field strength in uT

C = (Mag'-b)*A; % Calibrated data

icmData.MagXCal = C(:,1);
icmData.MagYCal = C(:,2);
icmData.MagZCal = C(:,3);

figure,
    subplot(4,1,1), plot(icmData.Time, Acc_rot(1,:), icmData.Time, Acc_rot(2,:), icmData.Time, Acc_rot(3,:))
    title('Accelerometer Data')
    subplot(4,1,2), plot(icmData.Time, Gyr_rot(1,:), icmData.Time, Gyr_rot(2,:), icmData.Time, Gyr_rot(3,:))
    title('Gyroscope Data')
    subplot(4,1,3), plot(icmData.Time, icmData.MagX, icmData.Time, icmData.MagY, icmData.Time, icmData.MagZ)
    title('Magnetometer Data')
    subplot(4,1,4), plot(icmData.Time, icmData.MagXCal, icmData.Time, icmData.MagYCal, icmData.Time, icmData.MagZCal)
    title('Calibrated Magnetometer Data')

figure,
    plot3(Mag(1,:),Mag(2,:),Mag(3,:),"LineStyle","none","Marker","o", ...
          "MarkerSize",2,"MarkerFaceColor","b"), hold on
    grid(gca,"on")
    plot3(C(:,1), C(:,2), C(:,3),"LineStyle","none","Marker","o", ...
      "MarkerSize",2,"MarkerFaceColor","r")
    axis equal
    xlabel("uT")
    ylabel("uT")
    zlabel("uT")
    legend("Uncalibrated Samples","Calibrated Samples","Location","southoutside")
    title("Uncalibrated vs Calibrated" + newline + "Magnetometer Measurements")
    hold off

%% Magnetic North Finding
% Extract accelerometer and magnetometer data from the table (assuming `icmData` is the table)
AccX = icmData.AccXRot;
AccY = icmData.AccYRot;
AccZ = icmData.AccZRot;
MagX = C(:, 1); % Use calibrated values
MagY = C(:, 2); % Use calibrated values
MagZ = C(:, 3); % Use calibrated values

% Initialize arrays to store roll, pitch, and yaw
numSamples = length(AccX);
roll = zeros(numSamples, 1);
pitch = zeros(numSamples, 1);
yaw = zeros(numSamples, 1);

% Compute roll, pitch, and yaw for each sample
for i = 1:numSamples
    % Extract accelerometer measurements
    ax = AccX(i);
    ay = AccY(i);
    az = AccZ(i);
    
    % Roll angle (phi) calculation
    roll(i) = atan2(ay, az);
    
    % Pitch angle (theta) calculation
    pitch(i) = atan2(-ax, sqrt(ay^2 + az^2));
    
    % Extract magnetometer measurements
    mx = MagX(i);
    my = MagY(i);
    mz = MagZ(i);
    
    % Normalize acceleration measurements for tilt correction
    normAcc = sqrt(ax^2 + ay^2 + az^2);
    ax = ax / normAcc;
    ay = ay / normAcc;
    az = az / normAcc;
    
    % Tilt compensation for magnetometer
    mx_comp = mx * cos(pitch(i)) + mz * sin(pitch(i));
    my_comp = mx * sin(roll(i)) * sin(pitch(i)) + my * cos(roll(i)) - mz * sin(roll(i)) * cos(pitch(i));
    
    % Yaw angle (psi) calculation with magnetic declination adjustment if needed
    yaw(i) = atan2(-my_comp, mx_comp);
end

% Convert angles from radians to degrees
roll = rad2deg(roll);
pitch = rad2deg(pitch);
yaw = rad2deg(yaw);

icmData.Roll = roll;
icmData.Pitch = pitch;
icmData.Yaw = yaw;

figure;
    subplot(3,1,1);
    plot(icmData.Time, roll); hold on;
    xline(calTime_);
    xline(calTime_+5);
    title('Roll Angle');
    xlabel('Time [s]');
    ylabel('Angle [degrees]');
    grid on;    
    subplot(3,1,2);
    plot(icmData.Time, pitch); hold on
    xline(calTime_);
    xline(calTime_+5);
    title('Pitch Angle');
    xlabel('Time [s]');
    ylabel('Angle [degrees]');
    grid on;    
    subplot(3,1,3);
    plot(icmData.Time, yaw); hold on
    xline(calTime_);
    xline(calTime_+5);
    title('Yaw Angle');
    xlabel('Time [s]');
    ylabel('Angle [degrees]');
    grid on;

initialMagCalData = icmData(icmData.Time >= calTime_ & icmData.Time <= calTime_+5, :);
initialMagNorthHeading = mean(initialMagCalData.Yaw);

disp(['Initial Magnetic North Heading: ', num2str(initialMagNorthHeading), ' [deg]'])

%% True North Finding
ate = [2024, 7, 19]; % Example value (year, month, day)

% Get the true north heading
true_north_heading = compensate_to_true_north(initialMagNorthHeading, init_latitude, init_longitude, init_altitude, date);

% Display the result
disp(['Initial True North Heading: ', num2str(true_north_heading), ' [deg]']);

%% Aligment to ML2
ml2DataFile = ['Datasets/7_19_2024/exp', num2str(exp)];
ifXYZPlot = true;

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
% disp(head(ml2Data))

projectedTrajectory = project_to_z_axis(ml2Data.X, ml2Data.Z, ml2Data.Y);

if ifXYZPlot
    figure('Name','Frame - XYZ'), hold on,
    plot3(ml2Data.X, ml2Data.Y, ml2Data.Z),
    plot3(projectedTrajectory(1,:), ml2Data.Y, projectedTrajectory(2,:)),
    xlabel('X [m]'), ylabel('Y [m]'), zlabel('Z [m]'),
    legend('ML2 Trajectory', 'ML2 Proj. Z-axis')
    view(0,0)   % XZ
    xlim([-1 1])
end

ml2Data.XProj = projectedTrajectory(1,:)';
ml2Data.YProj = ml2Data.Y;
ml2Data.ZProj = projectedTrajectory(2,:)';

% Left Handed XYZ Unity World Coordinate Frame (UWC-Frame):
UWC_left = [ml2Data.XProj, ml2Data.YProj, ml2Data.ZProj]';

offset_Deg = true_north_heading+180;

% Unity Aligment to Target
R_y_Offset = [cosd(offset_Deg) 0 sind(offset_Deg); 0 1 0; -sind(offset_Deg) 0 cosd(offset_Deg)];
UWC_left = R_y_Offset*UWC_left;

UWC_right = [1 0 0; 0 1 0; 0 0 -1] * UWC_left;

ENU = [1 0 0; 0 0 -1; 0 1 0] * UWC_right;

wgs84 = wgs84Ellipsoid('meter');
[X,Y,Z] = enu2ecef(ENU(1,:), ENU(2,:), ENU(3,:), init_latitude, init_longitude, init_altitude, wgs84);
ECEF = [X', Y', Z'];
lla = ecef2lla(ECEF, 'WGS84');

figure('Name','Google Maps'),
    map_plot(1) = plot_google_map('MapType','hybrid','MapScale', 1,'Scale',1.5,'ShowLabels',0); hold on
    map_plot(2) = plot(goal_longitude, goal_latitude, 'sk', 'MarkerSize', 4, 'MarkerFaceColor', 'k');
    map_plot(3) = plot(ml2Data.Lon, ml2Data.Lat, 'b', 'LineWidth', 2.5);
    map_plot(4) = plot(lla(:,2), lla(:,1), 'r', 'LineWidth', 2.5);
    legend([map_plot(2) map_plot(3) map_plot(4)],'$LLA_{Goal}$', '$LLA_{\overline{Aligned}}$', '$LLA_{Aligned}$', 'Interpreter', 'latex');
    xlabel('Longitude ($^{\circ}$)'), ylabel('Latitude ($^{\circ}$)'), axis equal
    title('Google Maps Trajectory');
    xlim([init_longitude-0.00015 init_longitude+0.00015]), ylim([init_latitude-0.0003 init_latitude+0.0003])
    set(gca, 'TickLabelInterpreter', 'latex');
    set(gca,'DefaultTextInterpreter', 'latex');
    set(gca,'DefaultAxesTickLabelInterpreter','latex');
    set(gca,'DefaultLegendInterpreter','latex');


%% FUNCTIONS:
function true_north_heading = compensate_to_true_north(magnetic_north_heading, latitude, longitude, altitude, date)
    % Function to compensate the magnetic north heading to get the true north heading
    % Inputs:
    %   magnetic_north_heading - Heading angle from magnetic north (in degrees)
    %   latitude - Latitude of the location (in degrees)
    %   longitude - Longitude of the location (in degrees)
    %   altitude - Altitude of the location (in meters)
    %   date - Date vector [year, month, day]
    % Output:
    %   true_north_heading - Compensated heading angle (in degrees)
    
    % Get the magnetic declination using wrldmagm
    [~, ~, dec] = wrldmagm(altitude, latitude, longitude, decyear(date));
    
    % Compensate the magnetic north heading
    true_north_heading = magnetic_north_heading + dec;
    
    % Ensure the heading is within the range [0, 360)
    true_north_heading = mod(true_north_heading, 360);
end

function projected_trajectory = project_to_z_axis(x, z, y)
    % Function to project a tilted trajectory in the x-z plane to align with the z-axis
    % Inputs:
    %   x - Array of x-coordinates of the trajectory
    %   z - Array of z-coordinates of the trajectory
    % Output:
    %   projected_trajectory - Array of projected z-coordinates (along the z-axis)
    
    % Ensure x and z are column vectors
    x = x(:);
    z = z(:);
    y = y(:);
    
    % Calculate the angle of the trajectory with respect to the z-axis
    delta_x = x(end) - x(1);
    delta_z = z(end) - z(1);
    theta = atan2(delta_x, delta_z); % Angle in radians
    
    % Create the rotation matrix to align the trajectory with the z-axis
    R = [cos(theta), -sin(theta) 0;
         sin(theta), cos(theta) 0;
         0 0 1];
    
    % Apply the rotation to each point in the trajectory
    trajectory = [x, z, y]';
    rotated_trajectory = R * trajectory;
    
    % The projected trajectory along the z-axis will be the second row of the rotated_trajectory
    projected_trajectory = [rotated_trajectory(1,:); rotated_trajectory(2,:)];
end