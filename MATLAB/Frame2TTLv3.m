%{
----------------------------------------------------------------------------

This file is part of the Sanworks Frame2TTL repository
Copyright (C) 2023 Sanworks LLC, Rochester, New York, USA

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
% the sensor output using the stream() function below.
%
% Thresholds for detection are in units of change in light intensity, computed by a 1ms sliding
% window average (20 samples measured 50μs apart). Using change in
% luminance instead of a simple luminance threshold improves performance on
% skipped frames, when the LCD's light-to-dark transition time is >1ms.
% Experiment with LightThreshold and DarkThreshold properties to achieve
% reliable frame detection. The default settings are optimal for the ipad4
% screen (https://www.adafruit.com/product/1751) when sync pixels are set
% to max intensity - rgb(255,255,255).
%
% Installation:
% 1. Connect the Frame2TTL device to the computer with a USB micro cable.
% 2. Connect the Frame2TTL device BNC connectors to an oscilloscope for
%    testing, or to your acquisition system input for data collection.
%
% - Create a Frame2TTL object with F = Frame2TTLv3('COMx') where COMx is your serial port string
% - Directly manipulate its fields to change threshold parameters on the device.
% - Run F.stream to see the optical sensor's streaming output (for testing purposes)
%
% - To use the device, set the patch of screen underneath the sensor to either full intensity or 0
%   intensity with alternating frames in your video stimulus. The Bpod PsychToolboxVideoPlayer plugin does this automatically.

classdef Frame2TTLv3 < handle
    properties
        Port % ArCOM Serial port
        LightThreshold = 75; % Avg light intensity change read from the sensor to indicate a dark -> light frame transition
        DarkThreshold  = -75; % Avg light intensity change read from the sensor to indicate a light -> dark frame transition
        AcquiredData
    end
    properties (Access = private)
        Timer % MATLAB timer (for reading data from the serial buffer during USB streaming)
        HWversion
        streaming = 0; % 0 if idle, 1 if streaming data
        gui = struct; % Handles for GUI elements
        nDisplaySamples = 200; % When streaming to plot, show up to 200 samples
        maxDisplayTime = 2; % When streaming to plot, show up to last 2 seconds
    end
    methods
        function obj = Frame2TTLv3(portString)
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
            obj.Port = ArCOMObject_Frame2TTL(portString, 480000000);
            obj.Port.write('C', 'uint8');
            response = obj.Port.read(1, 'uint8');
            if response ~= 218
                error('Could not connect =( ')
            end
            obj.Port.write('#', 'uint8');
            pause(.25);
            if obj.Port.bytesAvailable == 0
                obj.HWversion = 1;
            else
                obj.HWversion = obj.Port.read(1, 'uint8');
            end
            obj.Port.write('T', 'uint8', [obj.LightThreshold obj.DarkThreshold], 'int16');
            obj.nDisplaySamples = 200;
        end
        function set.LightThreshold(obj, thresh)
            obj.Port.write('T', 'uint8', [thresh obj.DarkThreshold], 'int16');
            obj.LightThreshold = thresh;
        end
        function set.DarkThreshold(obj, thresh)
            obj.Port.write('T', 'uint8', [obj.LightThreshold thresh], 'int16');
            obj.DarkThreshold = thresh;
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
            obj.Port.write('D', 'uint8');
            pause(3);
            obj.DarkThreshold = obj.Port.read(1, 'int16');
        end
        function setLightThreshold_Auto(obj)
            obj.Port.write('L', 'uint8');
            pause(3);
            obj.LightThreshold = obj.Port.read(1, 'int16');
        end
        function stream(obj)
            obj.AcquiredData = uint16(zeros(1,1000000));
            obj.streaming = 1;
            obj.gui.DisplayIntensities = nan(1,obj.nDisplaySamples);
            obj.gui.DisplayTimes = nan(1,obj.nDisplaySamples);
            obj.gui.Fig  = figure('name','Sensor Stream','numbertitle','off', 'MenuBar', 'none', 'Resize', 'off', 'CloseRequestFcn', @(h,e)obj.endAcq());
            obj.gui.Plot = axes('units','normalized', 'position',[.2 .2 .65 .65]); ylabel('Sensor value (bits)', 'FontSize', 18); xlabel('Time (s)', 'FontSize', 18);
            set(gca, 'xlim', [0 obj.maxDisplayTime], 'ylim', [0 65536]);
            Xdata = nan(1,obj.nDisplaySamples); Ydata = nan(1,obj.nDisplaySamples);
            obj.gui.OscopeDataLine = line([Xdata,Xdata],[Ydata,Ydata]);
            drawnow;
            obj.gui.DisplayPos = 1;
            obj.gui.AcquiredDataPos = 1;
            obj.gui.SweepStartTime = 0; 
            obj.Port.write(['S' 1], 'uint8');
            obj.Timer = timer('TimerFcn',@(h,e)obj.updatePlot(), 'ExecutionMode', 'fixedRate', 'Period', 0.05, 'Tag', ['F2TTL_' obj.Port.PortName]);
            start(obj.Timer);
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
            stop(obj.Timer);
            delete(obj.Timer);
            obj.Timer = [];
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
                Div = 500;
                Times = (obj.gui.DisplayPos:obj.gui.DisplayPos+nIntensities-1)/Div;
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
    end
end