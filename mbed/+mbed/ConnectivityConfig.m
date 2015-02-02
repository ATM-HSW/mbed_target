classdef ConnectivityConfig < rtw.connectivity.Config
%CONNECTIVITYCONFIG PIL configuration class for Arduino_Maple
%
%   CONNECTIVITYCONFIG(COMPONENTARGS) creates instances of MAKEFILEBUILDER,
%   LAUNCHER, RTIOSTREAMHOSTCOMMUNICATOR and collects them together into a
%   connectivity configuration class for PIL.
%
%   See also RTW.CONNECTIVITY.CONFIG, RTW.CONNECTIVITY.MAKEFILEBUILDER,
%   RTW.MYPIL.TARGETAPPLICATIONFRAMEWORK, RTW.MYPIL.LAUNCHER,
%   RTW.CONNECTIVITY.RTIOSTREAMHOSTCOMMUNICATOR, RTWDEMO_CUSTOM_PIL
    
%   Copyright 2008-2014 The MathWorks, Inc.
%             2014 Dr. Olaf Hagendorf, HS Wismar
    
    methods
        function this = ConnectivityConfig(componentArgs)
            
			disp('ConnectivityConfig called');
            
			% An executable framework specifies additional source files and
			% libraries required for building the PIL executable
            targetApplicationFramework = mbed.TargetApplicationFramework(componentArgs);
            
            % Filename extension for executable on the target system
            exeExtension = '.hex';
            
            % Create an instance of MakefileBuilder; this works in
            % conjunction with your template makefile to build the PIL
            % executable
            builder = rtw.connectivity.MakefileBuilder(componentArgs, targetApplicationFramework, exeExtension);
            
            % Launcher
            launcher = mbed.Launcher(componentArgs, builder);
            
            % File extension for shared libraries (e.g. .dll on Windows)
            sharedLibExt='.dll';

            % Evaluate name of the rtIOStream shared library
            rtiostreamLib = ['rtiostreamserial' sharedLibExt];
            
            communicator = mbed.Communicator(componentArgs, launcher, rtiostreamLib);
            communicator.setTimeoutRecvSecs(5);





            if ispc
                sl_ext = '.dll'; 
            else
                sl_ext = '.so'; 
            end
            libSerial = ['rtiostreamserial' sl_ext];
 
            PIL_Speed = mbed.Prefs.getPILSpeed;
            rtIOStreamOpenArgs = {libSerial,'-port', ['\\.\' mbed.Prefs.getComPort], '-baud', num2str(PIL_Speed)};                                                     
            
            communicator.setOpenRtIOStreamArgList(rtIOStreamOpenArgs); 
            
            % call super class constructor to register components
            this@rtw.connectivity.Config(componentArgs, builder, launcher, communicator);

            % The timer API is changing so we need to check whether to use the 
            % new or old style
            if mbed.ConnectivityConfig.isOldStyleTimerApi
                % Register a timer if the execution profiling infrastructure is available. For
                % PIL simulations, a file execProfile.mat is created in the pil
                % sub-folder of the build directory. This file contains execution
                % time measurements for the component that you are running in PIL
                % simulation mode.
                if ~isempty(?rtw.connectivity.Timer)
                    timer = mbed.TimerOldStyle(targetApplicationFramework);
                    this.setTimer(timer);
                end
            else
                timer = mbed.Timer;
                this.setTimer(timer);
            end
        end
    end
    
    methods (Static = true)
        function isOldStyle = isOldStyleTimerApi
            h = new_system;
            try
                get_param(h,'CodeExecutionProfiling');
                isOldStyle=false;
            catch exc
                if any(strcmp(exc.identifier, {'Simulink:SL_ParamUnknown', ...
                        'Simulink:Commands:ParamUnknown'}))
                    isOldStyle=true;
                else
                    rethrow(exc);
                end
            end
            close_system(h);
        end
    end
end
