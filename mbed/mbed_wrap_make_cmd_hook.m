function makeCmd = mbed_wrap_make_cmd_hook(args)
%MAKE_MBED wrap_make_cmd hook

%   Copyright 2009-2014 The MathWorks, Inc.
%             2014 Dr. Olaf Hagendorf, HS Wismar

if ispc
    disp('ispc');
	%postprocessingMaple(args.modelName, 0, 0, 0);
    mbedPath = get_param(args.modelName,'MbedPath');
    % gccPath = get_param(args.modelName,'GccPath');
    gccPath ='';
    a=find(mbedPath, filesep);
    d=[mbedPath(1:a(1)) ':'];
    args.make = fullfile(mbedPath, 'make.exe');
    args.makeCmd = ['cmd /C ' d ' & cd ' mbedPath ' & make.exe sketch GCCROOT=' gccPath ' SIMULINK_MODEL=embeddedcoder'];

    makeCmd = setup_for_default(args);
else
    disp('no ispc');
    makeCmd = fullfile(matlabroot,'bin',lower(computer),'gmake');
    pcMakeCmd = '%MBED_ROOT%/hardware/tools/avr/utils/bin/make';
    len=length(pcMakeCmd);
    assert(strncmp(args.makeCmd, pcMakeCmd, len), 'makeCmd must default to PC Make command');
    makeCmd = [makeCmd args.makeCmd(len+1:end)];
end
