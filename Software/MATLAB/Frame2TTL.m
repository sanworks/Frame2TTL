%{
----------------------------------------------------------------------------

This file is part of the Sanworks Frame2TTL repository
Copyright (C) Sanworks LLC, Rochester, New York, USA

----------------------------------------------------------------------------

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, version 3.

This program is distributed  WITHOUT ANY WARRANTY and without even the
implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
%}

% Frame2TTLv3 is a system to measure the instant when a computer monitor updates its display.
% It uses a light sensor that outputs light intensity as an analog signal. The
% voltage of the signal encodes light intensity. You can gain intuition for
% the sensor output using the streamUI() function below.
%
% Thresholds for detection are in units of change in light intensity, computed by a 1ms sliding
% window average (20 samples measured 50μs apart). Using change in
% luminance instead of a simple luminance threshold improves performance on
% skipped frames.
%
% Hardware Installation:
% 1. Connect the Frame2TTL device to the computer with a USB micro cable.
% 2. Connect the Frame2TTL device BNC connectors to an oscilloscope for
%    testing, or to your acquisition system input for data collection.
%
% Usage:
% - F = Frame2TTL('COMx'); % where COMx is your serial port string
% - F.LightThreshold = 100; % Set the dark --> light detection threshold
% - F.DarkThreshold = -100; % Set the light--> dark detection threshold
% - F.setDarkThreshold_Auto; % Automatically sets the dark threshold.
%                            % To be run while the pixels under the sensor
%                            are WHITE (standard sensor) or half pixel intensity (IBL sensor)
% - F.setLightThreshold_Auto; % Automatically sets the light threshold.
%                            % To be run while the pixels under the sensor
%                            are BLACK
% - Measurements = F.readSensor(nSamples) % Returns raw sensor
%   measurements. Units = bits in range 0 (no light) - 2^16 (sensor saturation)
%
% - F.streamUI; % Launches a GUI to view the optical sensor's streaming output (for testing purposes)
%
% - To use the device, alternate the patch of pixels underneath the sensor
%   between white and black with each frame in your video stimulus. For
%   white, use maximum pixel intensity (standard sensor) or half pixel intensity (IBL sensor).

classdef Frame2TTL < handle
    properties
        Port % ArCOM Serial port
        HardwareVersion
        FirmwareVersion
        LightThreshold  % Threshold to indicate a dark -> light frame transition
        DarkThreshold   % Threshold to indicate a light -> dark frame transition
        DetectMode      % Determines how light signal is processed to detect sync patch transitions.
                        % 0: detect when raw luminance measurement (0-65535) exceeds threshold
                        % 1 (default): avg sample-wise change in luminance exceeds threshold

        AcquiredData    % A struct with sensor data acquired on most recent call to streamUI()
    end
    properties (Access = private)
        currentFirmware = 4; % Latest firmware
        minSupportedFirmware = 3; % Minimum supported firmware
        readTimer % MATLAB timer (for reading data from the serial buffer during USB streaming)
        streaming = 0; % 0 if idle, 1 if streaming data
        gui = struct; % Handles for GUI elements
        nDisplaySamples = 200; % When streaming to plot, show up to 200 samples
        maxDisplayTime = 2; % When streaming to plot, show up to last 2 seconds
        initialized = false; % Set to true after the constructor finishes executing
        thresholdDatatype = 'int16'; % Datatype of threshold (will be set to match firmware version)
        activationMargin = 1000; % In DetectMode 0, thresholds are initially inactive until the
                                 % sensor value exceeds threshold by this amount (units = bits)
    end
    methods
        function obj = Frame2TTL(portString)
            % Destroy any orphaned timers from previous instances
            T = timerfindall;
            for i = 1:length(T)
                thisTimer = T(i);
                thisTimerTag = get(thisTimer, 'tag');
                if strcmp(thisTimerTag, ['F2TTL_' portString])
                    warning('off');
                    delete(thisTimer);
                    warning('on');
                end
            end

            % Initialize USB Serial connection
            obj.Port = ArCOMObject_Frame2TTL(portString, 12000000);
            obj.Port.write('C', 'uint8');
            response = obj.Port.read(1, 'uint8');
            if response ~= 218
                error('Could not connect =( ')
            end

            % Read firmware version
            obj.Port.write('F', 'uint8');
            pause(.25)
            if obj.Port.bytesAvailable == 0 % Firmware v1 does not respond to command 'F'
                error(['Old Frame2TTL firmware detected. Please update to firmware v'...
                    num2str(obj.minSupportedFirmware) ' or newer.'])
            end
            obj.FirmwareVersion = obj.Port.read(1, 'uint8');
            if obj.FirmwareVersion < obj.minSupportedFirmware
                error(['Old Frame2TTL firmware detected, v' num2str(obj.FirmwareVersion) '.' char(10) ...
                    'Please update to firmware v' num2str(obj.currentFirmware) '.'])
            end
            if obj.FirmwareVersion < obj.currentFirmware
                warning(['Old Frame2TTL firmware detected, v' num2str(obj.FirmwareVersion) '.'... 
                    char(10) 'Please update to firmware v' num2str(obj.currentFirmware) ' at your earliest convenience.'])
            end
            if obj.FirmwareVersion > obj.currentFirmware
                error(['Future Frame2TTL firmware detected, v' num2str(obj.FirmwareVersion) '.' char(10)...
                    'This software expects v' num2str(obj.currentFirmware) '.' char(10)...
                    'Update Frame2TTL software from: https://github.com/sanworks/Frame2TTL' char(10) ...
                    'or downgrade firmware to v' num2str(obj.currentFirmware)])
            end

            % Read hardware version
            obj.Port.write('#', 'uint8');
            obj.HardwareVersion = obj.Port.read(1, 'uint8');
            if obj.HardwareVersion > 2 % If Frame2TTL v3, set baud rate accordingly
                obj.Port = [];
                obj.Port = ArCOMObject_Frame2TTL(portString, 480000000);
            end

            % Set default thresholds
            if obj.FirmwareVersion > 3
                obj.initialized = true;
                obj.thresholdDatatype = 'int32';
            else
                obj.thresholdDatatype = 'int16';
            end
            obj.DetectMode = 1;
            obj.setThresholds2Default;

            if obj.FirmwareVersion < 4 % Earlier firmware versions use a single op to set thresholds
                obj.Port.write('T', 'uint8', [obj.LightThreshold obj.DarkThreshold], 'int16');
                obj.initialized = true;
            end
            
            % Set default threshold activation margin (for use in DetectMode 0)
            if obj.FirmwareVersion > 3
                obj.Port.write('G', 'uint8', obj.activationMargin, 'uint32');
            end
        end

        function set.LightThreshold(obj, thresh)
            if obj.initialized
                if obj.DetectMode == 0
                    maxThresh = 65535-obj.activationMargin;
                    if thresh >= obj.DarkThreshold
                        error('In DetectMode 0, LightThreshold cannot be set higher than DarkThreshold.')
                    end
                    if thresh < obj.activationMargin || thresh > 65535
                        error(['In DetectMode 0, thresholds must be in range [' num2str(obj.activationMargin)... 
                               ', ' num2str(maxThresh) '].'])
                    end
                elseif obj.DetectMode == 1
                    if thresh <= 0
                        error('In DetectMode 1, LightThreshold cannot be set <= 0.')
                    end
                end
                if obj.FirmwareVersion < 4
                    obj.Port.write('T', 'uint8', [thresh obj.DarkThreshold], 'int16');
                else
                    obj.Port.write('T', 'uint8', thresh, 'int32');
                end
            end
            obj.LightThreshold = thresh;
        end

        function set.DarkThreshold(obj, thresh)
            if obj.initialized
                if obj.DetectMode == 0
                    if thresh <= obj.LightThreshold
                        error('In DetectMode 0, DarkThreshold cannot be set lower than LightThreshold.')
                    end
                    maxThresh = 65535-obj.activationMargin;
                    if thresh < 0 || thresh > maxThresh
                        error(['In DetectMode 0, thresholds must be in range [' nu2mstr(obj.activationMargin)... 
                               ', ' num2str(maxThresh) '].'])
                    end
                elseif obj.DetectMode == 1
                    if thresh >= 0
                        error('In DetectMode 1, DarkThreshold cannot be set >= 0.')
                    end
                end
                if obj.FirmwareVersion < 4
                    obj.Port.write('T', 'uint8', [obj.LightThreshold thresh], 'int16');
                else
                    obj.Port.write('K', 'uint8', thresh, 'int32');
                end
            end
            obj.DarkThreshold = thresh;
        end

        function set.DetectMode(obj, newMode)
            originalMode = obj.DetectMode;
            if obj.FirmwareVersion > 3
                obj.Port.write(['M' newMode], 'uint8');
                obj.DetectMode = newMode;
                if newMode ~= originalMode
                    obj.setThresholds2Default;
                end
            else
                obj.DetectMode = newMode;
            end
        end

        function Value = readSensor(obj, varargin) % Optional argument = nSamples (contiguous)
            nSamples = 1;
            if nargin > 1
                nSamples = varargin{1};
            end
            obj.Port.write('V', 'uint8', nSamples, 'uint32');
            Value = obj.Port.read(nSamples, 'uint16');
        end

        function setDarkThreshold_Auto(obj)
            if obj.DetectMode == 0
                error(['Automatic threshold detection is only available for DetectMode 1.' char(10)...
                    'In DetectMode 0, set thresholds manually, guided by streamUI().'])
            end
            obj.Port.write('D', 'uint8');
            pause(3);
            obj.DarkThreshold = obj.Port.read(1, obj.thresholdDatatype);
        end

        function setLightThreshold_Auto(obj)
            if obj.DetectMode == 0
                error(['Automatic threshold detection is only available for DetectMode 1.' char(10)...
                    'In DetectMode 0, set thresholds manually, guided by streamUI().'])
            end
            obj.Port.write('L', 'uint8');
            pause(3);
            obj.LightThreshold = obj.Port.read(1, obj.thresholdDatatype);
        end

        function streamUI(obj)
            obj.AcquiredData = uint16(zeros(1,1000000));
            obj.streaming = 1;
            obj.gui.DisplayIntensities = nan(1,obj.nDisplaySamples);
            obj.gui.DisplayTimes = nan(1,obj.nDisplaySamples);
            obj.gui.Fig  = figure('name','Sensor Stream','numbertitle','off', 'MenuBar', 'none', 'Resize', 'off', 'CloseRequestFcn', @(h,e)obj.endAcq());
            obj.gui.Plot = axes('units','normalized', 'position',[.2 .2 .65 .65]); ylabel('Sensor value (bits)', 'FontSize', 18); xlabel('Time (s)', 'FontSize', 18);
            set(gca, 'xlim', [0 obj.maxDisplayTime], 'ylim', [0 65536]);
            Xdata = nan(1,obj.nDisplaySamples); Ydata = nan(1,obj.nDisplaySamples);
            obj.gui.OscopeDataLine = line([Xdata,Xdata],[Ydata,Ydata]);
            if obj.DetectMode == 0
                obj.gui.LightThreshLine = line([0,obj.maxDisplayTime],[obj.LightThreshold,obj.LightThreshold],...
                    'Color', [1 0.8 0], 'LineStyle', '--');
                obj.gui.DarkThreshLine = line([0,obj.maxDisplayTime],[obj.DarkThreshold,obj.DarkThreshold],...
                    'Color', [0.5 0.5 0.5], 'LineStyle', '--');
            end

            drawnow;
            obj.gui.DisplayPos = 1;
            obj.gui.AcquiredDataPos = 1;
            obj.gui.SweepStartTime = 0;
            obj.Port.write(['S' 1], 'uint8');
            obj.readTimer = timer('TimerFcn',@(h,e)obj.updatePlot(), 'ExecutionMode', 'fixedRate', 'Period', 0.05, 'Tag', ['F2TTL_' obj.Port.PortName]);
            start(obj.readTimer);
        end

        function delete(obj)
            if obj.streaming == 1
                obj.endAcq();
            end
            obj.Port = []; % Trigger the ArCOM port's destructor function (closes and releases port)
        end
    end

    methods (Access = private)
        function endAcq(obj)
            try
                stop(obj.readTimer);
                delete(obj.readTimer);
                obj.readTimer = [];
            catch
            end
            obj.Port.write(['S' 0], 'uint8');
            obj.streaming = 0;
            delete(obj.gui.Fig);
            pause(.1);
            if obj.Port.bytesAvailable > 0
                obj.Port.read(obj.Port.bytesAvailable, 'uint8');
            end
            obj.AcquiredData = obj.AcquiredData(1:obj.gui.AcquiredDataPos);
        end

        function updatePlot(obj)
            BytesAvailable = obj.Port.bytesAvailable;
            if BytesAvailable > 1
                nBytesToRead = floor(BytesAvailable/2)*2;
                NewIntensities = obj.Port.read(nBytesToRead, 'uint8');
                nIntensities = length(NewIntensities)/2;
                NewIntensities = typecast(NewIntensities, 'uint16');
                obj.AcquiredData(obj.gui.AcquiredDataPos:obj.gui.AcquiredDataPos+nIntensities-1) = NewIntensities;
                obj.gui.AcquiredDataPos = obj.gui.AcquiredDataPos + nIntensities;
                SamplingRate = 1000;
                Times = (obj.gui.DisplayPos:obj.gui.DisplayPos+nIntensities-1)/SamplingRate;
                DisplayTime = (Times(end)-obj.gui.SweepStartTime);
                obj.gui.DisplayPos = obj.gui.DisplayPos + nIntensities;
                if DisplayTime >= obj.maxDisplayTime
                    obj.gui.DisplayIntensities(1:obj.gui.DisplayPos) = NaN;
                    obj.gui.DisplayTimes(1:obj.gui.DisplayPos) = NaN;
                    obj.gui.DisplayPos = 1;
                    obj.gui.SweepStartTime = 0;
                else
                    SweepTimes = Times-obj.gui.SweepStartTime;
                    obj.gui.DisplayIntensities(obj.gui.DisplayPos-nIntensities+1:obj.gui.DisplayPos) = NewIntensities;
                    obj.gui.DisplayTimes(obj.gui.DisplayPos-nIntensities+1:obj.gui.DisplayPos) = SweepTimes;
                end
                set(obj.gui.OscopeDataLine,'xdata',[obj.gui.DisplayTimes, obj.gui.DisplayTimes], 'ydata', [obj.gui.DisplayIntensities, obj.gui.DisplayIntensities]); drawnow;
            end
            pause(.0001);
        end

        function setThresholds2Default(obj)
            switch obj.DetectMode
                case 0
                    obj.DarkThreshold = 30000;
                    obj.LightThreshold = 20000;
                case 1
                    switch obj.HardwareVersion
                        case 2
                            obj.LightThreshold = 100;
                            obj.DarkThreshold = -150;
                        case 3
                            obj.LightThreshold = 75;
                            obj.DarkThreshold = -75;
                    end
            end
        end
    end
end