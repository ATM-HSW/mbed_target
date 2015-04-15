function ret = postprocessingMbed(modelName, bBuildApplication, bDownloadApplication, bGetMakeCmd)
%disp('postprocessingMbed');
ret = '';
mbedPath = get_param(modelName,'MbedPath');
a=find(mbedPath, filesep);
d=[mbedPath(1:a(1)) ':'];
sourcePath = mbedPath;
%comPort = get_param(modelName,'ComPort');

% Get the current configuration
lCodeGenFolder = Simulink.fileGenControl('getConfig').CodeGenFolder;
[modelFolder,modelName,ext] = fileparts( which (bdroot));
srcFolder = fullfile(lCodeGenFolder, [modelName '_slprj']);
cfg = Simulink.fileGenControl('getConfig');
disp(['start copying files from ' srcFolder ' to "' sourcePath '"']);

if ~exist(sourcePath,'dir')
    mkdir(sourcePath);
end
if ~exist(sourcePath,'dir')
    error(['could not access or create ' sourcePath]);
end

try
    disp(['copy *.c to ' sourcePath]);
    [status,message,messageid] = copyfile('*.c',[sourcePath 'cc'],'f');
    if status==0 & ~isequal(messageid, 'MATLAB:COPYFILE:FileDoesNotExist')
        disp(message);
    end
catch err
    disp(err);
end

try
    disp(['copy *.cpp to ' sourcePath]);
    [status,message,messageid] = copyfile('*.cpp',sourcePath,'f');
    if status==0
        disp(message);
    end
catch err
    disp(err);
end

try
    disp(['copy *.h to ' sourcePath]);
    [status,message,messageid] = copyfile('*.h',sourcePath,'f');
    if status==0
        disp(message);
    end
catch err
    disp(err);
end

% if bBuildApplication
%     cmd = (['cmd /C ' d ' & cd ' mbedPath ' & make.exe sketch GCCROOT=' gccPath ' SIMULINK_MODEL=embeddedcoder']);
%     disp(cmd);
%     system(cmd);
% end

% if bGetMakeCmd
% end

% if bDownloadApplication
%     try
%         cmd = (['cmd /C ' d ' & cd ' mbedPath ' & make.exe reset GCCROOT=' gccPath ' PYTHONROOT=' pythonPath ' COMPORT=' comPort ' SIMULINK_MODEL=embeddedcoder']);
%         disp(cmd);
%         system(cmd);
%     catch  err
%         disp(err);
%     end
%     
%     getkeywait(5);
%     
%     try
%         cmd = (['cmd /C ' d ' & cd ' mbedPath ' & make.exe install_flash GCCROOT=' gccPath ' SIMULINK_MODEL=embeddedcoder']);
%         disp(cmd);
%         system(cmd);
%     catch  err
%         disp(err);
%     end
% end

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

%PilReq = updatePrefs('get','stm32f4PilMode');
%if (strcmp(get_param(modelName,'GenCodeOnly'),'off') && isequal(PilReq,0) )
%    if (strcmp(get_param(modelName,'DownloadApplication'),'on'))
%        download(modelName);
%    end
%    if (strcmp(get_param(modelName,'OpenIDE'),'on'))
%        openide(modelName);
%    end
%end
end
