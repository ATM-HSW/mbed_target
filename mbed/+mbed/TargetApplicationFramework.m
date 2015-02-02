classdef TargetApplicationFramework < rtw.pil.RtIOStreamApplicationFramework
%TARGETAPPLICATIONFRAMEWORK is a skeleton application framework for PIL
%
%   The TARGETAPPLICATIONFRAMEWORK allows you to specify additional files needed
%   to build an application for the target environment. These files may include
%   code for hardware initialization as well as device driver code for a
%   communications channel. 
%
%   This is a skeleton class that you can make into a full implementation of
%   TARGETAPPLICATIONFRAMEWORK by uncommenting the lines tagged with
%   "UNCOMMENT".  This is explained further in the demo RTWDEMO_CUSTOM_PIL.
%
%   See also RTW.PIL.RTIOSTREAMAPPLICATIONFRAMEWORK, RTWDEMO_CUSTOM_PIL
 
%   Copyright 2008-2014 The MathWorks, Inc.
    
    methods
        % constructor
        function this = TargetApplicationFramework(componentArgs)
            error(nargchk(1, 1, nargin, 'struct'));
            % call super class constructor
            this@rtw.pil.RtIOStreamApplicationFramework(componentArgs);
            
            % INSERT YOUR CODE HERE TO CUSTOMIZE THE CONNECTIVITY
            % CONFIGURATION FOR YOUR TARGET
            
           % To build the PIL application you must specify a main.c file.       
           % The following PIL main.c files are provided and can be             
           % added to the application framework via the "addPILMain"                
           % method:                                                            
           %                                                                    
           % 1) A main.c adapted for on-target PIL and suitable                 
           %    for most PIL implementations. Select by specifying              
           %    'target' argument to "addPILMain" method.                       
           %                                                                    
           % 2) A main.c adapted for host-based PIL such as the                 
           %    "mypil" host example. Select by specifying 'host'               
           %    argument to "addPILMain" method.                                
           this.addPILMain('target');                                             
                                                                                
           % Additional source and library files to include in the build        
           % must be added to the BuildInfo property                            
                                                                                
           % Get the BuildInfo object to update                                 
           buildInfo = this.getBuildInfo;                                       
                                                                                
           % Add device driver files to implement the target-side of the        
           % host-target rtIOStream communications channel                      
           rtiostream_src_path = fileparts(fileparts(mfilename('fullpath')));
           buildInfo.addSourceFiles('rtiostream_serial.cpp', rtiostream_src_path); 
           buildInfo.addSourcePaths(rtiostream_src_path); 
                                                                                 
        end
    end
end
