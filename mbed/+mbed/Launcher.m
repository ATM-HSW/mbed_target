classdef Launcher < rtw.connectivity.Launcher
%LAUNCHER launches a PIL or real-time application on Arduino_Maple
%
%   See also RTW.CONNECTIVITY.LAUNCHER, RTWDEMO_CUSTOM_PIL
    
%   Copyright 2008-2014 The MathWorks, Inc.
%             2014 Dr. Olaf Hagendorf, HS Wismar
  
    methods
        % constructor
        function this = Launcher(componentArgs, builder)
            error(nargchk(2, 2, nargin, 'struct'));
            % call super class constructor
            this@rtw.connectivity.Launcher(componentArgs, builder);
        end
        
        % destructor
        function delete(this) %#ok
        end
        
        % Start the application
        function startApplication(this)
            % get name of the executable file
            hexFile = this.getBuilder.getApplicationExecutable;
            postprocessingMaple(modelName, 0, 1, 'on'));
            disp('### Starting the PIL simulation')
        end
        
        % Stop the application
        function stopApplication(~)
            disp('### Stopping PIL simulation')
        end
    end
end
