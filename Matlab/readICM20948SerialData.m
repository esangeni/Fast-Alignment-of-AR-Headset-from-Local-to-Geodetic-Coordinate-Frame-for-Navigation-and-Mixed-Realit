function readICM20948SerialData
    close all; clear all; clc;
    % Define serial port and baud rate
    serialPort = 'COM5'; % Change this to the appropriate port for your ESP32
    baudRate = 115200;

    % Open serial connection
    serialObj = serialport(serialPort, baudRate);

    % Define the output .txt file
    outputFile = 'exp2000_dataICM20948.txt';
    fid = fopen(outputFile, 'w');

    % Create a stop flag and start time
    stopFlag = false;
    startTime = tic;

    % Create a figure for the dialog box
    fig = figure('Name', 'Data Collection', 'NumberTitle', 'off', ...
                 'Position', [100, 100, 300, 150], 'CloseRequestFcn', @closeFig);

    % Create a stop button
    stopButton = uicontrol(fig, 'Style', 'pushbutton', 'String', 'Stop', ...
                           'Position', [100, 50, 100, 40], ...
                           'Callback', @stopCallback);

    % Create a text display for the elapsed time
    timeText = uicontrol(fig, 'Style', 'text', 'String', 'Elapsed time: 0 s', ...
                         'Position', [50, 100, 200, 20]);

    % Create a timer to update the elapsed time display
    timerObj = timer('TimerFcn', @updateTime, 'Period', 1, 'ExecutionMode', 'fixedSpacing');
    start(timerObj);

    disp('Collecting data...')

    % Collect data until the stop button is pressed
    while ~stopFlag
        if serialObj.NumBytesAvailable > 0
            dataLine = readline(serialObj); % Read one line of data from serial port
            fprintf(fid, '%s\n', dataLine); % Write data line to file
        end
        pause(0.01); % Small delay to prevent overload
    end

    % Stop and delete the timer
    stop(timerObj);
    delete(timerObj);

    % Close the serial connection and the file
    fclose(fid);
    clear serialObj;
    
    % Close the figure
    if isvalid(fig)
        close(fig);
    end    

    disp('Data collection complete.');

    % Nested callback function for the stop button
    function stopCallback(~, ~)
        stopFlag = true;
    end

    % Nested timer callback function to update the elapsed time
    function updateTime(~, ~)
        elapsedTime = toc(startTime);
        set(timeText, 'String', sprintf('Elapsed time: %.1f s', elapsedTime));
    end

    % Nested callback function for closing the figure
    function closeFig(~, ~)
        stopFlag = true;
        delete(gcf);
    end
end



% % Define serial port and baud rate
% serialPort = 'COM6'; % Change this to the appropriate port for your ESP32
% baudRate = 115200;
% 
% % Open serial connection
% serialObj = serial(serialPort, 'BaudRate', baudRate);
% fopen(serialObj);
% 
% % Define the output .txt file
% outputFile = 'exp100_dataICM20948.txt';
% fid = fopen(outputFile, 'w');
% 
% % Define the duration for data collection (in seconds)
% duration = 5; % Change this to your preferred data collection duration
% startTime = tic;
% 
% disp('Collecting data...')
% 
% % Collect data for the specified duration
% while toc(startTime) < duration
%     if serialObj.BytesAvailable > 0
%         dataLine = fgetl(serialObj); % Read one line of data from serial port
%         fprintf(fid, '%s\n', dataLine); % Write data line to file
%     end
% end
% 
% % Close the serial connection and the file
% fclose(serialObj);
% delete(serialObj);
% fclose(fid);
% 
% disp('Data collection complete.')