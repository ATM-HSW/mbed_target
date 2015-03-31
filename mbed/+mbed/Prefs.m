classdef Prefs
%PREFS gives access to mbed preferences file
%
%   This is an undocumented class. Its methods and properties are likely to
%   change without warning from one release to the next.
%
%   Copyright 2009-2014 The MathWorks, Inc.
%             2014 Dr. Olaf Hagendorf, HS Wismar

    methods (Static, Access=public)
%%
        function setPILSpeed(speed)
            mbed.Prefs.setPref('PILSpeed', speed);
        end

%%
        function speed = getPILSpeed
            speed = mbed.Prefs.getPref('PILSpeed');
            if isempty(speed)
                speed = 9600;    % take default speed
            end
        end
%%
        function setArduinoPath(toolPath)

            if ~exist('toolPath', 'var') || ~ischar(toolPath)
               nl = sprintf('\n');
               error('RTW:mbed:invalidArduinoPath', ...
                      ['Arduino path must be a string, e.g.' nl ...
                       '   mbed.Prefs.setArduinoPath(''c:\\arduino-1.0.5'')']);
            end

            if ~exist(toolPath,'dir')
                error('RTW:mbed:invalidArduinoPath', 'The specified folder (%s) does not exist', toolPath);
            end

            if ~exist(fullfile(toolPath, 'arduino.exe'), 'file')
                error('RTW:mbed:invalidArduinoPath', 'The specified folder (%s) does not contain arduino.exe', toolPath);
            end

            % remove trailing backslashes
            toolPath = regexprep(toolPath, '\\+$', '');

            % Alternate form of path to handle spaces
            altPath = RTW.transformPaths(toolPath, 'pathType', 'alternate');

            mbed.Prefs.setPref('ArduinoPath', altPath);

            % The board data is tied to a specific version of Arduino IDE, so if we change
            % the Arduino path (possibly to a different IDE) the existing data may not be valid
            mbed.Prefs.setPref('ArduinoBoard', []);

            mbed.Prefs.setPref('ArduinoBoardName', '');
        end

%%
        function toolPath = getArduinoPath
            toolPath = mbed.Prefs.getPref('ArduinoPath');
            % check validity of path (in case the folder got deleted between
            % after setArduinoPath and before getArduinoPath)
            if ~exist(toolPath,'dir')
                nl = sprintf('\n');
                error('RTW:mbed:invalidArduinoPath', ...
                      ['Arduino path is unspecified or invalid.' nl ...
                       'Specify a valid path using mbed.Prefs.setArduinoPath, e.g.' nl ...
                       '   mbed.Prefs.setArduinoPath(''c:\\arduino-1.0.5'')']);
            end
        end

        %%
        function setMbedPath(toolPath) 
            
            if ~exist('toolPath', 'var') || ~ischar(toolPath) 
               nl = sprintf('\n');
               error('RTW:mbed:invalidMbedPath', ['Mbed path must be a string, e.g.' nl '   mbed.Prefs.setMbedPath(''c:\\GitHub\\mbed'')']);
            end
                
            if ~exist(toolPath,'dir')
                error('RTW:mbed:invalidMbedPath', 'The specified folder (%s) does not exist', toolPath);
            end
                        
            if ~exist(fullfile(toolPath, 'mbed'), 'dir')
                error('RTW:mbed:invalidMbedPath', 'The specified folder (%s) does not contain library folder', toolPath);
            end
            
            % remove trailing backslashes
            toolPath = regexprep(toolPath, '\\+$', '');            
            
            % Alternate form of path to handle spaces
            % altPath = RTW.transformPaths(toolPath,'pathType','alternate');            
            
            mbed.Prefs.setPref('MbedPath', toolPath);
            %disp(mbed.Prefs.getPref('MbedPath'));
        end
        
        %%
        function toolPath = getMbedPath
            toolPath = mbed.Prefs.getPref('MbedPath');
            % check validity of path (in case the folder got deleted between
            % after setMbedPath and before getMbedPath)
            if ~exist(toolPath,'dir') 
                nl = sprintf('\n');
                error('RTW:mbed:invalidMbedPath', ...
                      ['Mbed path is unspecified or invalid: "' toolPath '"' nl ...
                       'Specify a valid path using mbed.Prefs.setMbedPath, e.g.' nl ...
                       '   mbed.Prefs.setMbedPath(''c:\\GitHub\\mbed'')']);
            end
        end


        %%
        function setMbedDrive(devicePath) 
            
%             if ~exist('devicePath', 'var') || ~ischar(devicePath) 
%                nl = sprintf('\n');
%                error('RTW:mbed:invalidMbedDrive', ...
%                       ['Mbed drive must be a string, e.g.' nl ...
%                        '   mbed.Prefs.setMbedDrive(''c:\\'')']);
%             end
%                 
%             if ~exist(devicePath,'dir')
%                 error('RTW:mbed:invalidMbedPath', 'The specified folder (%s) does not exist', devicePath);
%             end
                        
            % remove trailing backslashes
            devicePath = regexprep(devicePath, '\\+$', '');            
            
            % Alternate form of path to handle spaces
            % altPath = RTW.transformPaths(devicePath,'pathType','alternate');            
            
            mbed.Prefs.setPref('MbedDrive', devicePath);
            %disp(mbed.Prefs.getPref('MbedPath'));
        end
        
        %%
        function devicePath = getMbedDrive
            devicePath = mbed.Prefs.getPref('MbedDrive');
            % check validity of path (in case the folder got deleted between
            % after setMbedDrive and before getMbedDrive)
%             if ~exist(devicePath,'dir') 
%                 nl = sprintf('\n');
%                 error('RTW:mbed:invalidMbedDrive', ...
%                       ['Mbed Drive is unspecified or invalid: "' devicePath '"' nl ...
%                        'Specify a valid path using mbed.Prefs.setMbedDrive, e.g.' nl ...
%                        '   mbed.Prefs.setMbedDrive(''E:\\'')']);
%             end
        end

        %%
        function setGccPath(toolPath) 
            
            if ~exist('toolPath', 'var') || ~ischar(toolPath) 
               nl = sprintf('\n');
               error('RTW:mbed:invalidGccPath', ...
                      ['Gcc path must be a string, e.g.' nl ...
                       '   mbed.Prefs.setGccPath(''C:\\GCC\\Sourcery_CodeBench_Lite_for_ARM_EABI\\bin'')']);
            end
                
            if ~exist(toolPath,'dir')
                error('RTW:mbed:invalidGccPath', 'The specified folder (%s) does not exist', toolPath);
            end
                        
            if ~exist(fullfile(toolPath, 'arm-none-eabi-gcc.exe'), 'file')
                error('RTW:mbed:invalidGccPath', 'The specified folder (%s) does not contain arm-none-eabi-gcc.exe', toolPath);
            end
            
            % remove trailing backslashes
            toolPath = regexprep(toolPath, '\\+$', '');            
            
            % Alternate form of path to handle spaces
            altPath = RTW.transformPaths(toolPath,'pathType','alternate');            
            
            mbed.Prefs.setPref('GccPath', altPath);           
        end
        
        %%
        function toolPath = getGccPath
            toolPath = mbed.Prefs.getPref('GccPath');
            % check validity of path (in case the folder got deleted between
            % after setGccPath and before getGccPath)
            if ~exist(toolPath,'dir') 
                nl = sprintf('\n');
                error('RTW:mbed:invalidGccPath', ...
                      ['Gcc path is unspecified or invalid.' nl ...
                       'Specify a valid path using mbed.Prefs.setGccPath, e.g.' nl ...
                       '   mbed.Prefs.setGccPath(''C:\\GCC\\Sourcery_CodeBench_Lite_for_ARM_EABI\\bin'')']);
            end
        end
         %%
        function setPythonPath(toolPath) 
            
            if ~exist('toolPath', 'var') || ~ischar(toolPath) 
               nl = sprintf('\n');
               error('RTW:mbed:invalidPythonPath', ...
                      ['Python path must be a string, e.g.' nl ...
                       '   mbed.Prefs.setPythonPath(''C:\\Python27'')']);
            end
                
            if ~exist(toolPath,'dir')
                error('RTW:mbed:invalidPythonPath', 'The specified folder (%s) does not exist', toolPath);
            end
                        
            if ~exist(fullfile(toolPath, 'python.exe'), 'file')
                error('RTW:mbed:invalidPythonPath', 'The specified folder (%s) does not contain python.exe', toolPath);
            end
            
            % remove trailing backslashes
            toolPath = regexprep(toolPath, '\\+$', '');            
            
            % Alternate form of path to handle spaces
            altPath = RTW.transformPaths(toolPath,'pathType','alternate');            
            
            mbed.Prefs.setPref('PythonPath', altPath);           
        end
        
        %%
        function toolPath = getPythonPath
            toolPath = mbed.Prefs.getPref('PythonPath');
            % check validity of path (in case the folder got deleted between
            % after setPythonPath and before getPythonPath)
            if ~exist(toolPath,'dir') 
                nl = sprintf('\n');
                error('RTW:mbed:invalidpythonPath', ...
                      ['Python path is unspecified or invalid.' nl ...
                       'Specify a valid path using mbed.Prefs.setPythonPath, e.g.' nl ...
                       '   mbed.Prefs.setPythonPath(''C:\\Python27'')']);
            end
        end
        
%%
        function setBoard(boardLabel)
            boardsFile = fullfile(mbed.Prefs.getArduinoPath(), 'hardware', 'arduino', 'boards.txt');

            if ~exist(boardsFile, 'file')
                nl = sprintf('\n');
                error('RTW:mbed:invalidArduinoPath', ...
                      ['Unable to find board specification file. Ensure that' nl ...
                       'the path to the Arduino IDE is set correctly, e.g.' nl ...
                       '  mbed.Prefs.setArduinoPath(''c:\arduino-1.0.5'')'] );
            end
            boards = mbed.Prefs.parseBoardsFile(boardsFile);
            if isempty(boards)
                error('RTW:mbed:invalidBoardSpecification', ...
                      'Unable to read board specification file (%s)', boardsFile);
            end
            
            parsedLines = regexp(boards, ['^(', boardLabel, '\..+)=([^$]+)$'],'tokens');
            % parsedLines = regexp(rawLines,'^([^=]+)=([^$]+)$','tokens');

            specifiedBoard = {};
            ind = 0;
            for i=1:numel(parsedLines)
                if ~isempty(parsedLines{i})
                    ind = ind + 1;
                    specifiedBoard(ind) = parsedLines{i};
                end
            end

            if isempty(specifiedBoard)
                msg = 'Specified board not found in configuration file';
                error('RTW:mbed:invalidBoardLabel', msg);
            end
            mbed.Prefs.setPref('ArduinoBoard', specifiedBoard);
            mbed.Prefs.setPref('ArduinoBoardName', boardLabel);
        end

        %%
        function [boardName] = getName            
            boardName = 'STM32F4Discovery'; %mbed.Prefs.getPref('ArduinoBoard');
        end

		%%
        function [boardLabel, allData] = getBoard
            board = 'discovery_f4'; %mbed.Prefs.getPref('ArduinoBoard');
            %boardLabel = mbed.Prefs.getPref('ArduinoBoardName');
            %board = mbed.Prefs.getPref('ArduinoBoard');
            %if isempty(board)
            %    nl = sprintf('\n');
            %    error('RTW:mbed:noBoardSpecification', ...
            %          ['Arduino board is not yet specified. ' nl ...
            %           'Specify the board using mbed.Prefs.setBoard, e.g.' nl ...
            %           '  mbed.Prefs.setBoard(''uno'') ']);
            %end
            boardLabel = board;
            if nargout == 2
                allData = board;
            end
        end
%%
        function mcu = getMCU
            %mcu = mbed.Prefs.getKey('mcu');
            mcu = 'STM32F407VG'; %board.build.mcu;
        end
%%
        function cpu = getCPU
            %cpu = mbed.Prefs.getKey('cpu');
            cpu = 'STM32F407VG'; %board.build.mcu;
        end
%%
        function uploadRate = getUploadRate
            uploadRate = mbed.Prefs.getKey('upload.speed$');
        end
%%
        function ret = getKey(key)
            [~, board] = mbed.Prefs.getBoard;
            ret = '';
            key = regexprep(key, '\.', '\\.');
            for i=1:numel(board)
                found = regexp(board{i}{1}, ['.*\.' key]);
                if found
                    ret = board{i}{2};
                    return
                end
            end
        end
%%
        function programmer = getProgrammer
            programmer =  mbed.Prefs.getKey('protocol');
        end
%%
        function cpu_freq = getCpuFrequency
            %cpu_freq = mbed.Prefs.getKey('f_cpu');
            cpu_freq = '168000000'; %int32(sscanf(board.build.f_cpu,'%u'));
        end
%%
        function port = getComPort
            port = mbed.Prefs.getPref('ComPort');
            if isempty(port)
                nl = sprintf('\n');
                msg = [
                    'The serial port must be set and you must have installed' nl ...
                    'the device drivers for your hardware. ' nl ...
                    ' 1. Install the drivers and connect the hardware. ' nl ...
                    ' 2. Identify the virtual serial (COM) port. You can do this through' nl ...
                    '    the Windows Device Manager, or by running mbed.Prefs.setComPort' nl ...
                    ' 3. Set the correct COM port using mbed.Prefs.setComPort' ...
                    ];
                error('RTW:mbed:invalidComPort', msg);
            end
        end
%%
        function setComPort(port)
            if ~exist('port', 'var') || ~ischar(port) || isempty(port)
                nl = sprintf('\n');
                error('RTW:mbed:invalidComPort', ...
                      ['Specify the COM port as a string. E.g.: ' nl ...
                       '   mbed.Prefs.setComPort(''COM8'') ']);
            end
            mbed.Prefs.setPref('ComPort', port);
        end
%%
        function ports = searchForComPort(regCmdOutput)
            ports='';

            if ispc
                if nargin < 1
                    regCmd=['reg query '...
                        'HKEY_LOCAL_MACHINE\HARDWARE\DEVICEMAP\SERIALCOMM'];
                    [~,regCmdOutput]=system(regCmd);
                end

                deviceName='\\Device\\(VCP\d|USBSER\d{3})';
                reg_sz = 'REG_SZ';
                portNum = 'COM\d+';
                expr = [deviceName '\s+' reg_sz '\s+(' portNum ')'];
                allPorts=regexp(regCmdOutput,expr,'tokens');
                if ~isempty(allPorts)
                    ports=cell(1, length(allPorts));
                    for j=1:length(allPorts)
                        ports{j}=allPorts{j}{2};
                    end
                end
            end
        end

        function timers = getTimersSpecs()
            % This function returns full specification for every available
            % timer on the current target.
            timers = { struct('max_tcnt', int32(256), 'prescalers', int32([1 8 32 64 128 256 1024])), ...
                       struct('max_tcnt', int32(65536), 'prescalers', int32([1 8 64 256 1024])) };
        end
    end
%%
    methods(Static,Access=private)

        function setPref(prefName, prefValue)
            prefGroup = 'ArduinoGeneric';
            setpref(prefGroup, prefName, prefValue);
        end

        function prefValue = getPref(prefName)
            prefGroup = 'ArduinoGeneric';
            if ispref(prefGroup,prefName)
                prefValue = getpref(prefGroup, prefName);
            else
                prefValue = '';
            end
        end

        function boards = parseBoardsFile(filename)
            boards = {};
            fid = fopen(filename, 'rt');
            if fid < 0,
                return;
            end
            txt = textscan(fid,'%s', 'commentstyle','#','delimiter','\n', 'multipledelimsasone',true);
            fclose(fid);
            boards = txt{1};
        end
    end
end

% LocalWords:  USBSER
