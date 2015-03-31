function makeCmd = mbed_wrap_make_cmd_hook(args)
%MAKE_MBED wrap_make_cmd hook

%   Copyright 2009-2014 The MathWorks, Inc.
%             2014-2015 Dr. Olaf Hagendorf, HS Wismar

if ispc
    %disp('ispc');
    mbedPath = get_param(args.modelName,'MbedPath');
    
    a=find(mbedPath, filesep);
    d=[mbedPath(1:a(1)) ':'];
    args.make = fullfile(mbedPath, 'make.exe');
    % create a batch file to call make with following steps:
    %  - copy generated make file (modelname.mk from gencodefolder to target folder and rename it to Makefile
    %  - change drive to target drive
    %  - change dir to target folder
    %  - call make.exe
    args.makeCmd = [['@echo off & copy "' fullfile(pwd, [args.modelName '.mk']) '" "' fullfile(mbedPath, 'Makefile') '"'] ' & ' 'cmd /C ' d ' & cd ' mbedPath ' & make.exe '];
    makeCmd = setup_for_default(args);
else
    %disp('no ispc');
    makeCmd = fullfile(matlabroot,'bin',lower(computer),'gmake');
    pcMakeCmd = '%MBED_ROOT%/hardware/tools/avr/utils/bin/make';
    len=length(pcMakeCmd);
    assert(strncmp(args.makeCmd, pcMakeCmd, len), 'makeCmd must default to PC Make command');
    makeCmd = [makeCmd args.makeCmd(len+1:end)];
end
