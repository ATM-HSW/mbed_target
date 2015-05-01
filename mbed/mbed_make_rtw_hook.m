function mbed_make_rtw_hook(hookMethod, modelName, rtwroot,templateMakefile,buildOpts,buildArgs)
% MBED_MAKE_RTW_HOOK

%   Copyright 1996-2014 The MathWorks, Inc.
%             2014-2015 Dr. Olaf Hagendorf, HS Wismar

  switch hookMethod
   case 'error'
    % Called if an error occurs anywhere during the build.  If no error occurs
    % during the build, then this hook will not be called.  Valid arguments
    % at this stage are hookMethod and modelName. This enables cleaning up
    % any static or global data used by this hook file.
    disp(['### Build procedure for model: ''' modelName ''' aborted due to an error.']);

   case 'entry'
    % Called at start of code generation process (before anything happens.)
    % Valid arguments at this stage are hookMethod, modelName, and buildArgs.
    %disp('entry');
    i_mbed_setup(modelName);

   case 'before_tlc'
    % Called just prior to invoking TLC Compiler (actual code generation.)
    % Valid arguments at this stage are hookMethod, modelName, and
    % buildArgs
    %disp('before_tlc');

   case 'after_tlc'
    % Called just after to invoking TLC Compiler (actual code generation.)
    % Valid arguments at this stage are hookMethod, modelName, and
    % buildArgs
    %disp('after_tlc');

    % Safely check if model contains property 'UseRTOS'
    param = get_param(modelName, 'ObjectParameters');
    if isfield(param,'UseRTOS')
        rtos = strcmp(get_param(gcs,'UseRTOS'),'off');
    else
        rtos = false;
    end
    
    if ~rtos    % Multitasking is possible with RTOS only
        % This check must be done after the model has been compiled otherwise
        % sample time may not be valid
        i_check_tasking_mode(modelName)
    end

   case 'before_make'
    % Called after code generation is complete, and just prior to kicking
    % off make process (assuming code generation only is not selected.)  All
    % arguments are valid at this stage.
    %disp('before_make');
    % if ( strcmp(get_param(gcs,'ParallelExecution'),'on') )
    %    args = get_param(modelName, 'RTWBuildArgs');
    %    args = [args, ' -j4'];  % run makefile in 4 threads
    %    set_param(modelName, 'RTWBuildArgs', args);
    % end     
    i_write_mbed_files();
    disp(sprintf(['\n### Code Format : %s'],buildOpts.codeFormat));%#ok

   case 'after_make'
    % Called after make process is complete. All arguments are valid at
    % this stage.
    %disp('after_make');
    if ( strcmp(get_param(gcs,'DownloadApplication'),'on') )
        downloadApplication = 1;
    else
        downloadApplication = 0;
    end
    downloadMethod = get_param(gcs,'DownloadMethod');

    if ~i_isPilSim && ~i_isModelReferenceBuild(modelName) && downloadApplication
        %disp('i_download');
        i_download(modelName, downloadApplication, downloadMethod)
    end

   case 'exit'
    % Called at the end of the build process.  All arguments are valid at this
    % stage.
    rtw = RTW.GetBuildDir(modelName);
    if i_isPilSim
        fileDis = fullfile(rtw.BuildDirectory, 'pil', 'disassembly.txt');
        fileMap = fullfile(rtw.BuildDirectory, 'pil', 'mapFile.map');
    else
        fileDis = fullfile(rtw.BuildDirectory, 'disassembly.txt');
        fileMap = fullfile(rtw.BuildDirectory, 'mapFile.map');
    end
    %fprintf('### Disassembling project code into <a href="matlab:edit %s">disassembly.txt</a>\n', fileDis);
    %fprintf('### Linker Map file <a href="matlab:edit %s">mapFile.map</a>\n', fileMap);
    
    disp(['### Successful completion of build procedure for model: ', modelName]);
  end
end


function i_mbed_setup(modelName)

    if ~i_isPilSim
        % Check that the main function will be generated using the correct .tlc file
        if bdIsLoaded(modelName) && ~i_isModelReferenceBuild(modelName)
            requiredSetting = 'mbed_file_process.tlc';
            assert(strcmp(get_param(modelName, 'ERTCustomFileTemplate'), requiredSetting),...
             'The model %s must have ERTCustomFileTemplate set to %s.', modelName, requiredSetting);
        end
    end

    % Check for C_INCLUDE_PATH
    if ~isempty(getenv('C_INCLUDE_PATH'))
        error('RTW:mbed:nonEmptyCIncludePath',...
              ['The environment variable C_INCLUDE_PATH is set. '...
               'This may conflict with the gcc for AVR. You should '...
               'clear this environment variable, e.g. by running '...
               'setenv(''C_INCLUDE_PATH'','''') from the MATLAB command '...
               'window.']);
    end

    disp(['### Starting mbed build procedure for ', 'model: ',modelName, ' Target: ', get_param(bdroot,'MbedTarget')]);

    if ~isempty(strfind(pwd,' ')) || ~isempty(strfind(pwd,'&'))
        error('RTW:mbed:pwdHasSpaces',...
              ['The current working folder, %s, contains either a space or ' ...
               'ampersand character. This is '...
               'not supported. You must change the current working folder to '...
               'a path that does not contain either of these characters.'], pwd);
    end

    % Display current settings in build log
    disp('###')
    disp('### mbed environment settings:')
    disp('###')
    fprintf('###     Name:           %s\n', get_param(bdroot,'MbedTarget'));
    fprintf('###     RTOS:           %s\n', get_param(bdroot,'UseMbedRTOS'));
    fprintf('###     mbed Drive:     %s\n', get_param(bdroot,'MbedDrive'));
    fprintf('###     Com Port:       %s\n', get_param(bdroot,'ComPort'));
    fprintf('###     Programmer:     %s\n', mbed.Prefs.getMbedProgrammer()); %get_param(bdroot,'MbedProgrammer'))
    disp('###')
end

function i_check_tasking_mode(modelName)
    % No support for multi tasking mode
    if ~i_isModelReferenceBuild(modelName)  &&  ~i_isPilSim
        solverMode = get_param(modelName,'SolverMode');
        st = get_param(modelName,'SampleTimes');
        if length(st)>1 && ~strcmp(solverMode,'SingleTasking')
            error('RTW:mbed:noMultiTaskingSupport',...
                  ['The multi-tasking solver mode is not supported for the real-time '...
                   'mbed target. '...
                   'In Simulation > Configuration Parameters > Solver you must select '...
                   '"SingleTasking" from the pulldown "Tasking mode for periodic sample '...
                   'times".']);
        end
    end
end

function i_write_mbed_files()

    lCodeGenFolder = Simulink.fileGenControl('getConfig').CodeGenFolder;
    [~,modelName,~] = fileparts( which (bdroot));
    buildAreaDstFolder = fullfile(lCodeGenFolder, [modelName '_slprj']);
    if ~exist(buildAreaDstFolder, 'dir')
      mkdir (buildAreaDstFolder);
    end
    
    target = get_param(bdroot,'MbedTarget');

    % Copy the mbed target into the build area
    srcFile = fullfile(mbed_getTargetRootPath(), 'targets', [target '.zip']);
    unzip(srcFile, buildAreaDstFolder);

end

function i_download(modelName, bFlash, flashMethod)
    lCodeGenFolder = Simulink.fileGenControl('getConfig').CodeGenFolder;
    srcname = fullfile(lCodeGenFolder, [modelName '_slprj'], [modelName '.bin']);
    if bFlash && strcmp(flashMethod,'mbed')
        destname = get_param(modelName,'MbedDrive');
        try
            disp(['copy ' srcname ' to ' destname]);
            [status,message,~] = copyfile(srcname,destname,'f');
            if status==0
                disp(message);
            end
        catch err
            disp(err);
        end
    elseif bFlash
        cmd = [fullfile(mbed_getTargetRootPath(), 'targets_flash', [flashMethod '.bat'])];
        cmd = [cmd ' ' '"' srcname '"'];
        cmd = [cmd ' ' '"' mbed_getTargetRootPath() '"'];
        cmd = [cmd ' ' '"' get_param(modelName,'MbedDrive') '"'];
        cmd = [cmd ' ' '"' get_param(modelName,'ComPort') '"'];
        system(cmd,'-echo')
    end
end

function isPilSim = i_isPilSim
    s = dbstack;
    isPilSim = false;
    for i=1:length(s)
        if strfind(s(i).name,'build_pil_target')
            isPilSim=true;
            break;
        end
    end
end

function isMdlRefBuild = i_isModelReferenceBuild(modelName)
    mdlRefTargetType = get_param(modelName, 'ModelReferenceTargetType');
    isMdlRefBuild = ~strcmp(mdlRefTargetType, 'NONE');
end
