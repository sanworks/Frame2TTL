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

function LoadFrame2TTLFirmware(port, varargin)
% Firmware load tool for Frame2TTL. This tool will load the latest stable
% firmware binary to the device.
%
% Args:
% port: The USB serial port of the device
% version (optional): The version number to load in range 1, 4294967295
%
% Usage: LoadFrame2TTLFirmware('COM3'); % Load the latest firmware to device on port COM3
%        LoadFrame2TTLFirmware('COM4', 2); % Load firmware version 2 to device on COM4

% Verify supported platform
if ~ismember(computer,{'PCWIN64', 'GLNXA64'})
    error(['Error: The Frame2TTL firmware updater is not yet available on %s.' char(10)...
        'Please load from source using the Arduino application.'])
end

% Check for udev rules on linux
if ~ismac && isunix && ~exist('/etc/udev/rules.d/00-teensy.rules','file')
    error(['Error: Cannot find teensy udev rules.' char(10) ...
        'Please follow instructions <a href="matlab:web(''https://www.pjrc.com/teensy/td_download.html'',''-browser'')">here</a> to install them.'])
end

% Location of firmware binaries
FirmwarePath = fileparts(mfilename('fullpath'));
thisFolder = fileparts(which('LoadFrame2TTLFirmware'));

% Define path for tycmd executable
switch computer
    case 'PCWIN64'
        tycmd = fullfile(FirmwarePath,'tycmd');
    case 'GLNXA64'
        tycmd = fullfile(FirmwarePath,'tycmd_linux64');
end

% Determine target firmware version
AllFiles = dir(FirmwarePath);
firmwareVersions = [];
for i = 3:length(AllFiles)
    FileExt = AllFiles(i).name(end-2:end);
    if strcmp(FileExt, 'hex')
        FileName = AllFiles(i).name(1:end-4);
        VerPos = strfind(FileName, 'v');
        thisVersion = FileName(VerPos+1:end);
        firmwareVersions(end+1) = str2double(thisVersion);
    end
end
targetVersion = max(firmwareVersions);
if nargin > 1
    requestedFirmwareVersion = varargin{1};
    if sum(firmwareVersions == requestedFirmwareVersion) > 0
        targetVersion = requestedFirmwareVersion;
    else
        error(['Firmware version ' num2str(requestedFirmwareVersion) ' could not be found.'])
    end
end

FileName = ['Frame2TTLFirmware_v' num2str(targetVersion) '.hex'];

if ~ispc && ~ismac
    try % Try to give the uploader execute permissions
        [OK, Msg] = system(['chmod a+x "' fullfile(thisFolder, 'tycmd_linux64') '"']);
        if ~isempty(Msg)
            warning(Msg)
        end
    catch
    end
end

% Kill existing Teensy processes
if ispc
    [x, y] = system('taskkill /F /IM teensy.exe');
elseif isunix
    [x, y] = system('killall teensy');
end
pause(.2);

% Resolve paths
firmwarePath = fullfile(thisFolder, FileName);
programPath = ['"' tycmd '" upload "' firmwarePath '" --board "@' port '"'];

% Load firmware and display result
disp('------Uploading new firmware------')
disp([FileName ' ==> ' port])
[~, msg] = system(programPath);
OK = 0;
if ~isempty(strfind(msg, 'Sending reset command')) || ~isempty(strfind(msg, 'Verify successful'))
    OK = 1;
end
if OK
    disp('----------UPDATE COMPLETE---------')
else
    disp('----------FAILED TO LOAD----------')
end
disp(' ');