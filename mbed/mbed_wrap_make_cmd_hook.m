function makeCmd = mbed_wrap_make_cmd_hook(args)
%MAKE_MBED wrap_make_cmd hook

%   Copyright 2009-2014 The MathWorks, Inc.
%             2014-2015 Dr. Olaf Hagendorf, HS Wismar

if ispc
    %disp('ispc');
    args.make = 'make.exe';
    % create a batch file to call make with following steps:
    %  - copy generated make file (modelname.mk) to Makefile
    %  - call make.exe
    lCodeGenFolder = Simulink.fileGenControl('getConfig').CodeGenFolder;
    [~,modelName,~] = fileparts( which (bdroot));
    buildAreaDstFolder = fullfile(lCodeGenFolder, [modelName '_slprj']);
    args.makeCmd = [['@echo off & copy "' fullfile(buildAreaDstFolder, [args.modelName '.mk']) '" "' fullfile(buildAreaDstFolder, 'Makefile') '"'] ' & make.exe'];
    makeCmd = setup_for_default(args);
else
    %disp('no ispc');
    makeCmd = fullfile(matlabroot,'bin',lower(computer),'gmake');
    pcMakeCmd = '%MBED_ROOT%/hardware/tools/avr/utils/bin/make';
    len=length(pcMakeCmd);
    assert(strncmp(args.makeCmd, pcMakeCmd, len), 'makeCmd must default to PC Make command');
    makeCmd = [makeCmd args.makeCmd(len+1:end)];
end
