%{
----------------------------------------------------------------------------

This file is part of the Sanworks Frame2TTL repository
Copyright (C) 2017 Sanworks LLC, Sound Beach, New York, USA

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

% Frame2TTL is a system to measure the instant when a computer monitor updates its display.
% It uses a light sensor that outputs light intensity as a square wave. The
% period of the square wave ("Sensor Value" in microseconds) encodes light intensity.
% Lower periods = higher frequency = higher light intensity. A typical
% value for a patch of screen whose pixels are at max intensity = 20us. A
% typical value for a patch of screen at minimum intensity = 500us. A
% typical value in total darkness = 20,000us. You can gain intuition for
% the sensor and its thresholds using the stream() function below.
%
% Installation:
% 1. Install ArCOM from https://github.com/sanworks/ArCOM
% 2. Connect the Frame2TTL device to the computer with a USB micro cable.
% 3. Connect the Frame2TTL device BNC connector to an oscilloscope for
%    testing, or to your acquisition system input for data collection. 
%
% - Create a Frame2TTL object with F = Frame2TTL('COMx') where COMx is your serial port string
% - Directly manipulate its fields to change threshold parameters on the device.
% - Run F.stream to see the optical sensor's streaming output (for testing purposes)
%
% - Optional: When you have found threshold values you are happy with, edit
%   their values in the Frame2TTL Arduino sketch, and upload it to the
%   device. This will cause the device to use those threshold values on
%   boot, and you will no longer need to use the MATLAB software to set the thresholds.
%
% - To use the device, set the patch of screen underneath the sensor to either full intensity or 0
%   intensity with alternating frames in your video stimulus. The Bpod
%   PsychToolboxMovieServer plugin does this automatically.

classdef Frame2TTL < handle
    properties
        Port % ArCOM Serial port
        LightThreshold = 100; % Light intensity read from the sensor to indicate a dark -> light frame transition
        DarkThreshold = 200; % Light intensity read from the sensor to indicate a light -> dark frame transition
        SlidingWindowFilterNsamples = 1;
    end
    properties (Access = private)
        streaming = 0; % 0 if idle, 1 if streaming data
        gui = struct; % Handles for GUI elements
        nDisplaySamples = 100; % When streaming to plot, show up to 100 samples
        maxDisplayTime = 2; % When streaming to plot, show up to last 2 seconds
    end
    methods
        function obj = Frame2TTL(portString)
            obj.Port = ArCOMObject_Frame2TTL(portString, 115200);
            obj.Port.write('C', 'uint8');
            response = obj.Port.read(1, 'uint8');
            if response ~= 218
                error('Could not connect =( ')
            end
        end
        function set.LightThreshold(obj, thresh)
            obj.Port.write('T', 'uint8', [thresh obj.DarkThreshold], 'uint16');
            obj.LightThreshold = thresh;
        end
        function set.DarkThreshold(obj, thresh)
            obj.Port.write('T', 'uint8', [obj.LightThreshold thresh], 'uint16');
            obj.DarkThreshold = thresh;
        end
        function Value = ReadSensorValue(obj)
            obj.Port.write('V', 'uint8');
            Value = obj.Port.read(1, 'uint32');
        end
        function set.SlidingWindowFilterNsamples(obj, nSamples)
            if (nSamples > 0) && (nSamples < 11)
                obj.Port.write('N', 'uint8', nSamples, 'uint16');
                obj.SlidingWindowFilterNsamples = nSamples;
            else
                error('Error setting sliding window filter, nSamples must be in the range 1 - 10.')
            end
        end
        function stream(obj)
            obj.streaming = 1;
            DisplayIntensities = nan(1,obj.nDisplaySamples);
            DisplayTimes = nan(1,obj.nDisplaySamples);
            obj.gui.Fig  = figure('name','Sensor Stream','numbertitle','off', 'MenuBar', 'none', 'Resize', 'off', 'CloseRequestFcn', @(h,e)obj.endAcq());
            obj.gui.Plot = axes('units','normalized', 'position',[.2 .2 .65 .65]); ylabel('Sensor value (us)', 'FontSize', 18); xlabel('Time (s)', 'FontSize', 18);
            set(gca, 'xlim', [0 obj.maxDisplayTime], 'ylim', [0 500]);
            Xdata = nan(1,obj.nDisplaySamples); Ydata = nan(1,obj.nDisplaySamples);
            obj.gui.HighThreshLine = line([0,obj.maxDisplayTime],[obj.LightThreshold,obj.LightThreshold], 'Color', [.6 .6 .6]);
            obj.gui.LowThreshLine = line([0,obj.maxDisplayTime],[obj.DarkThreshold,obj.DarkThreshold], 'Color', [.3 .3 .3]);
            obj.gui.OscopeDataLine = line([Xdata,Xdata],[Ydata,Ydata]);
            DisplayPos = 1;
            drawnow;
            obj.Port.write(['S' 1], 'uint8');
            SweepStartTime = 0;
            while obj.streaming
                BytesAvailable = obj.Port.bytesAvailable;
                if BytesAvailable > 3
                    nBytesToRead = floor(BytesAvailable/4)*4;
                    NewIntensities = obj.Port.read(nBytesToRead, 'uint8');
                    nIntensities = length(NewIntensities)/4;
                    NewIntensities = typecast(NewIntensities, 'uint32');
                    Times = (DisplayPos:DisplayPos+nIntensities-1)/100;
                    DisplayTime = (Times(end)-SweepStartTime);
                    DisplayPos = DisplayPos + nIntensities;
                    if DisplayTime >= obj.maxDisplayTime
                        DisplayIntensities(1:DisplayPos) = NaN;
                        DisplayTimes(1:DisplayPos) = NaN;
                        DisplayPos = 1;
                        SweepStartTime = 0;
                    else
                        SweepTimes = Times-SweepStartTime;
                        DisplayIntensities(DisplayPos-nIntensities+1:DisplayPos) = NewIntensities;
                        DisplayTimes(DisplayPos-nIntensities+1:DisplayPos) = SweepTimes;
                    end
                    set(obj.gui.OscopeDataLine,'xdata',[DisplayTimes, DisplayTimes], 'ydata', [DisplayIntensities, DisplayIntensities]); drawnow;
                end
                pause(.0001);
            end
        end
        function delete(obj)
            obj.Port = []; % Trigger the ArCOM port's destructor function (closes and releases port)
        end
    end
    methods (Access = private)
        function endAcq(obj)
            obj.Port.write(['S' 0], 'uint8');
            obj.streaming = 0;
            delete(obj.gui.Fig);
            if obj.Port.bytesAvailable > 0
                obj.Port.read(obj.Port.bytesAvailable, 'uint8');
            end
        end
        function degrees = pos2degrees(obj, pos)
            degrees = (double(pos)/1024)*360;
        end
        function pos = degrees2pos(obj, degrees)
            pos = uint16((degrees/360)*1024);
        end
    end
end