s = serialport("COM4", 115200);
configureTerminator(s, "LF");  % Configure line ending

% Initialize figure
figure('Name', 'Real-Time Serial Plot');
h = animatedline('Color', 'blue', 'LineWidth', 1.5);
ax = gca;
ax.YGrid = 'on';
ax.XGrid = 'on';
title('Serial Data Plot');
xlabel('Time (s)');
ylabel('Value');

% Initialize time
startTime = datetime('now');

% Create a cleanup object to ensure port is closed
cleanupObj = onCleanup(@() cleanup(s));

% Main loop
while true
    try
        % Read data from serial port
        if s.NumBytesAvailable > 0
            % Read line of data
            data = str2double(readline(s));
            
            % Get current time
            t = datetime('now') - startTime;
            
            % Add points to animation
            addpoints(h, seconds(t), data);
            
            % Update axes limits
            ax.XLim = [seconds(t)-10, seconds(t)+2];
            
            % Auto-adjust Y axis every 100 points
            % if mod(length(h.XData), 100) == 0
            %     ax.YLim = [min(h.YData)-10, max(h.YData)+10];
            % end
            
            % Update display
            drawnow limitrate
        end
    catch ME
        % Error handling
        fprintf('Error: %s\n', ME.message);
        break;
    end
end
