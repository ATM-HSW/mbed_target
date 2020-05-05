%  MbedTarget Simulink target
%  Copyright (c) 2014-2017 Dr.O.Hagendorf , HS Wismar
%
%  Licensed under the Apache License, Version 2.0 (the "License");
%  you may not use this file except in compliance with the License.
%  You may obtain a copy of the License at
%
%      http://www.apache.org/licenses/LICENSE-2.0
%
%  Unless required by applicable law or agreed to in writing, software
%  distributed under the License is distributed on an "AS IS" BASIS,
%  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
%  See the License for the specific language governing permissions and
%  limitations under the License.

function mbed_make_rtw_hook(hookMethod, modelName, rtwroot, templateMakefile, buildOpts, buildArgs)
% MBED_MAKE_RTW_HOOK

%   Copyright 1996-2014 The MathWorks, Inc.
%             2014-2017 Dr. Olaf Hagendorf, HS Wismar

switch hookMethod
    case 'error'
        % Called if an error occurs anywhere during the build.  If no error occurs
        % during the build, then this hook will not be called.  Valid arguments
        % at this stage are hookMethod and modelName. This enables cleaning up
        % any static or global data used by this hook file.
        disp(['### Build procedure for model: ''' modelName ''' aborted due to an error.(' hookMethod ')']);
        
    case 'entry'
        % Called at start of code generation process (before anything happens.)
        % Valid arguments at this stage are hookMethod, modelName, and buildArgs.
        %disp('entry');
        tic;
        i_mbed_setup(modelName);
        %disp('entry out');
        
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
        i_write_mbed_files();
        %disp(sprintf(['\n### Code Format : %s'],buildOpts.codeFormat));%#ok
        %disp('before_make out');
        
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
        %disp('after_make out');
        
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
        toc
end
end


function i_mbed_setup(modelName)

if ~i_isPilSim
    % Check that the main function will be generated using the correct .tlc file
    if bdIsLoaded(modelName) && ~i_isModelReferenceBuild(modelName)
        requiredSetting = 'mbed_ert_file_process.tlc';
        assert(strcmp(get_param(modelName, 'ERTCustomFileTemplate'), requiredSetting),...
            'The model %s must have ERTCustomFileTemplate set to %s.', modelName, requiredSetting);
    end
end

mbedautodetect = get_param(bdroot,'MbedlsAutodetect');
if isequal(mbedautodetect, 'on')
    [MbedDrive, ComPort, MbedTarget5] = mbed_mbedls();
    if strlength(MbedDrive) > 1
        set_param(bdroot,'MbedDrive', MbedDrive);
        set_param(bdroot,'ComPort', ComPort);
        set_param(bdroot,'MbedTarget5', MbedTarget5);
    else
        error('mbedls did not detect a target or more than one target is connected');
    end
end

mbedtarget = get_param(bdroot,'MbedTarget5');
disp(['### Starting mbed build procedure for ', 'model: ',modelName, ' Target: ', mbedtarget]);

if ~isempty(strfind(pwd,' ')) || ~isempty(strfind(pwd,'&'))
    error('RTW:mbed:pwdHasSpaces',...
        ['The current working folder, %s, contains either a space or ' ...
        'ampersand character. This is '...
        'not supported. You must change the current working folder to '...
        'a path that does not contain either of these characters.'], pwd);
end

libraryversion='';
try
%     fid=fopen(fullfile(mbed_getTargetRootPath(), 'targets', 'mbed-os', 'mbed.h'), 'r');
%     text = textscan(fid,'%s','Delimiter','','endofline','');
%     text = text{1}{1};
%     fclose(fid);
%     libraryversion = regexp(text,'MBED_LIBRARY_VERSION[\s\.=]+(\d+)','tokens');
%     libraryversion=string(libraryversion);

    fid=fopen(fullfile(mbed_getTargetRootPath(), 'targets', 'mbed-os', 'platform', 'mbed_version.h'), 'r');
    text = textscan(fid,'%s','Delimiter','','endofline','');
    text = text{1}{1};
    fclose(fid);
    major = regexp(text,'MBED_MAJOR_VERSION[\s\.=]+(\d+)','tokens');
    minor = regexp(text,'MBED_MINOR_VERSION[\s\.=]+(\d+)','tokens');
    patch = regexp(text,'MBED_PATCH_VERSION[\s\.=]+(\d+)','tokens');
    libraryversion=string(major) + "." + string(minor) + "." + string(patch);
catch ME
end

% Display current settings in build log
disp('###')
disp('### mbed environment settings:')
disp('###')
fprintf('###     Name:            %s\n', mbedtarget);
fprintf('###     Version:         Mbed-OS v%s\n', libraryversion);
fprintf('###     RTOS:            %s\n', get_param(bdroot,'UseMbedRTOS'));
fprintf('###     Fixed step size: %ss\n', get_param(bdroot,'FixedStep'));
fprintf('###     # of Make jobs:  %s\n', get_param(bdroot,'MakeJobs'));
fprintf('###     mbed Drive:      %s\n', get_param(bdroot,'MbedDrive'));
fprintf('###     Com Port:        %s\n', get_param(bdroot,'ComPort'));
fprintf('###     Programmer:      %s\n', mbed.Prefs.getMbedProgrammer()); %get_param(bdroot,'MbedProgrammer'))
additionalProjectFiles = get_param(bdroot,'MbedAddProjectFiles');
if ~isequal(additionalProjectFiles,'none')
    [buildAreaDstFolder, folder] = mbed_getTargetDestFolder();
fprintf('###     add. project file: %s\n', fullfile(buildAreaDstFolder, [folder '.xxx']));
end
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

    [~,modelName,~] = fileparts( which (bdroot));

    target = get_param(bdroot,'MbedTarget5');
    
    %    pathstr = mbed_getTargetRootPath();
    %    buildAreaDstFolder = fullfile(pathstr,'targets',[modelName '_slprj']);
    [buildAreaDstFolder, folder] = mbed_getTargetDestFolder();
    additionalProjectFiles = fullfile(buildAreaDstFolder, [folder '.*']);
    if exist(buildAreaDstFolder, 'dir')
        try
            delete(fullfile(buildAreaDstFolder,'BUILD','*.*'));
        catch
        end
        try
            rmdir(fullfile(buildAreaDstFolder,'BUILD'), 's');
        catch
        end
        try
            delete(fullfile(buildAreaDstFolder,'*.c'));
        catch
        end
        try
            delete(fullfile(buildAreaDstFolder,'*.cpp'));
        catch
        end
        try
            delete(fullfile(buildAreaDstFolder,'*.h'));
        catch
        end
        try
            delete(fullfile(buildAreaDstFolder,'*.mk'));
        catch
        end
        try
            delete(fullfile(buildAreaDstFolder,'*.bat'));
        catch
        end
        try
            delete(fullfile(buildAreaDstFolder,'*.json'));
        catch
        end
        try
            delete(fullfile(buildAreaDstFolder,'.mbedignore'));
        catch
        end
        try
            delete(additionalProjectFiles);
        catch
        end
    else
        mkdir (buildAreaDstFolder);
    end
    mkdir(fullfile(buildAreaDstFolder,'BUILD'));
    
    % copy generated code/header/makefile into the build area under
    % mbed_target/targets/modelName+'_slprj'
    lCodeGenFolder = Simulink.fileGenControl('getConfig').CodeGenFolder;
    path = fullfile(lCodeGenFolder, [modelName '_slprj']);
    try
        srcFile = fullfile(path, '*.c');
        copyfile(srcFile, buildAreaDstFolder);
    catch
    end
    try
        srcFile = fullfile(path, '*.cpp');
        copyfile(srcFile, buildAreaDstFolder);
    catch
    end
    try
        srcFile = fullfile(path, '*.h');
        copyfile(srcFile, buildAreaDstFolder);
    catch
    end
    try
        srcFile = fullfile(path, '*.mk');
        copyfile(srcFile, buildAreaDstFolder);
    catch
    end
    try
        srcFile = fullfile(path, 'Makefile');
        copyfile(srcFile, buildAreaDstFolder);
    catch
    end
    
    targetconfig = get_param(bdroot,'MbedAppConfig');
    if ~isempty(targetconfig)
        path = fullfile(mbed_getTargetRootPath(), 'targets', 'appconfig', targetconfig);
        try
            srcFile = fullfile(path, '*.*');
            copyfile(srcFile, buildAreaDstFolder);
        catch
        end
    end
    
    % generate make file with mbed tools
    oldpath=cd(buildAreaDstFolder);

    destFile = fullfile(buildAreaDstFolder,'.mbedignore');
    ret = exist(destFile,'file');
    srcFile = fullfile(mbed_getTargetRootPath(), 'targets', 'appconfig', '.mbedignore');
    if ret == 0
        copyfile(srcFile, buildAreaDstFolder);
    elseif ret == 2
        system(['copy ' srcFile '+' fullfile(buildAreaDstFolder,'.mbedignore') ' ' fullfile(buildAreaDstFolder,'.mbedignore.tmp')]);
        try
            delete(fullfile(buildAreaDstFolder,'.mbedignore'));
        catch
        end
        try
            movefile(fullfile(buildAreaDstFolder,'.mbedignore.tmp'), fullfile(buildAreaDstFolder,'.mbedignore'));
        catch
        end
    end

    [~,cmdout]=system(['python ..\mbed-os\tools\project.py -m ' target ' -i simulink --source . --source ..\mbed-os --source ..\libraries']);

    % generate additional project config files for µVision, IAR, ...
    additionalProjectFiles = 'uvision6'; % get_param(bdroot,'MbedAddProjectFiles');
    if ~isequal(additionalProjectFiles,'none')
      [~,cmdout]=system(['python ..\mbed-os\tools\project.py -m ' target ' -i ' additionalProjectFiles ' --source . --source ..\mbed-os --source ..\libraries']);
    end
    cd(oldpath);
    disp(cmdout);

end

function i_download(modelName, bFlash, flashMethod)
    pathstr = mbed_getTargetDestFolder();
    srcname = fullfile(pathstr, 'BUILD', [modelName '.bin']);

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
