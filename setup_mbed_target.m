function setup_customtarget_mbed

addpath(fullfile(pwd,'mbed'),fullfile(pwd,'blocks'),fullfile(pwd,'blocks','mex'),fullfile(pwd,'blocks','slx'),fullfile(pwd,'mbed','jsonlab-1.5'));

result = savepath;
if result==1
    nl = char(10);
    msg = [' Unable to save updated MATLAB path (<a href="http://www.mathworks.com/support/solutions/en/data/1-9574H9/index.html?solution=1-9574H9">why?</a>)' nl ...
        ' Exit MATLAB, right-click on the MATLAB icon, select "Run as administrator", and re-run setup_customtarget_mbed.m' nl ...
        ];
    error(msg);
else
    disp(' Saved updated MATLAB path');
end

%% Register PIL/ExtMode communication interface
sl_refresh_customizations

%% setting the default configuration for a new Simulink model
mbed.Prefs.setMbedTarget('DISCO_F407VG');
mbed.Prefs.setMbedDrive('D:');
mbed.Prefs.setMbedDownload('on');
mbed.Prefs.setMbedRTOS('off');
mbed.Prefs.setMbedProgrammer('mbed');


%% set user specific variables:
% setenv('PATH', [getenv('PATH') ';C:\Program Files (x86)\GNU Tools ARM Embedded\4.9 2015q1\bin']);
% setenv('PATH', [getenv('PATH') ';C:\Program Files (x86)\GNU Tools ARM Embedded\make-3.81\bin']);
% setenv('PATH', [getenv('PATH') ';C:\Program Files (x86)\STMicroelectronics\STM32 ST-LINK Utility\ST-LINK Utility']);
% setenv('PATH', [getenv('PATH') ';C:\Python27']);
if exist('setup_userprefs.m','file')>0
    disp ' Setting user prefs';
    setup_userprefs;
end

ret=dir('blocks\mex');
ret1=dir('blocks\mex\sourcen');
if(size(ret,1)<=4 && size(ret1,1)>2)
    disp('did not found mexw64 files: start compiling');
    choice = txtmenu('Do you want to compile the the missing S-Function? C/C++ compiler has to be setup for this.','yes','no');
    if choice==0
        cd blocks
        mex_compile_all
        cd ..
    end
end


[ok, where, version] = getGCC();
if ok
    disp(' ');
    disp('found gcc version:');
    disp(version);
    disp('in folder:');
    disp(where);
    disp('All versions newer than 4.8 are OK for mbed_target');
else
    error('Can not find arm-none-eabi-gcc.exe. Please install GNU ARM Embedded Toolchain 4.8-2014-q3-update or better and add the bin folder to System or User Path');
end

% [ok, where, version] = getMake();
% if ok
%     disp(' ');
%     disp('found make version:');
%     disp(version);
%     disp('in folder:');
%     disp(where);
%     disp('All versions newer than 3.81 are OK for mbed_target');
% else
%     error('Can not find make.exe. Please install GNU make 3.81vfor Windows or better and add the bin folder to System or User Path');
% end

[okPython, where, version] = getPython();
if okPython
    disp(' ');
    disp('found Python version');
    disp(version);
    disp('in folder:');
    disp(where);
    disp('All versions newer than 2.7.9 are OK for mbed_target and mbed5 compatibility');
else
    warning('Can not find python.exe. When you want to use mbed5 targets, please install Python 2.7.9 or newer');
end
if okPython
    choice = txtmenu('Do you want to install the mbed os requirements with "pip install -r requirements.txt"?','yes','no');
    if choice==0
        disp('Please check the output of the installation.');
        pathstr = mbed_getTargetRootPath();
        newpath = fullfile(pathstr,'targets','mbed-os');
        oldpath=cd(newpath);
        system('pip install -r requirements.txt');
        cd(oldpath);
    end
end
if okPython
    %[status,out]=system('targets\mbed-os\tools\project.py -S targets');
    [a,b,c]=mbed_getTargets('mbed-os 5');
    if(c>0)
        disp(' ');
        disp('supported mbed os 5 boards:')
        disp(b);
        disp(['found ' int2str(c) ' boards with "project.py -S targets"']);
    end
end
% [ok, where, version] = getMbed();
% if ok
%     disp(' ');
%     disp('found mbed-cli version');
%     disp(version);
%     disp('in folder:');
%     disp(where);
%     disp('All versions newer than 0.9.10 are OK for mbed_target and mbed5 compatibility');
% else
%     disp(' ');
%     disp('Can not find mbed-cli.');
%     if okPython
%         choice = txtmenu('Do you want to install mbed-cli?','yes','no');
%         if choice==0
%             disp('Please check the output of the installation. Or rerun this setup.');
%             system('pip install mbed-cli');
%         else
%             warning('When you want to use mbed5 targets, please install mbed with "pip install mbed-cli"');
%         end
%     else
%         warning('When you want to use mbed5 targets, please install mbed with "pip install mbed-cli"');
%     end
% end

[okMbedls, where, version] = getMbedls();
if okMbedls
    disp(' ');
    disp('found mbed-ls version');
    disp(version);
    disp('in folder:');
    disp(where);
    disp('All versions newer than 1.2.9 are OK for mbed_target and mbed5 compatibility');
    disp('When it is to old please call: "pip install mbed-ls --upgrade" at a Windows commandline')
else
    disp(' ');
    disp('Can not find mbed-ls.');
    if okPython
        choice = txtmenu('Do you want to install mbed-ls?','yes','no');
        if choice==0
            disp('Please check the output of the installation. Or rerun this setup.');
            system('pip install mbed-ls');
        else
            warning('When you want to use mbed5 targets, please install mbed with "pip install mbed-ls"');
        end
    else
        warning('When you want to use mbed5 targets, please install mbedls with "pip install mbed-ls"');
    end
end
if okMbedls
    [status,out]=system('mbedls');
    if(size(out,2)>1)
        disp(' ');
        disp('found a mbed board:')
        disp(out);
    end
end

try
    mbed_getTargetRootPath();
catch
    disp(' ');
    error('Checking mbed_target Matlab path setting. Please check the path if multiple mbed_targets are configured.')
end
end


function [ok, where, version] = getPython
[status,out]=system('where python');
ok = false;
where = '';
version = '';
if status ~= 0
    return;
end
lines=strfind(out,10);
where = out(1:lines(1)-1);
[status,out]=system('python --version');
if status ~= 0 OR isempty(strfind(out,'2.7'))
    ok = false;
else
    ok = true;
end
if ok
    version = out(strfind(out,'2.7'):end-1);
end
end

function [ok, where, version] = getGCC
[status,out]=system('where arm-none-eabi-gcc');
ok = false;
where = '';
version = '';
if status ~= 0
    return;
end
lines=strfind(out,10);
where = out(1:lines(1)-1);
[status,out]=system('arm-none-eabi-gcc --version');
if status ~= 0
    ok = false;
else
    ok = true;
end
if ok
    version = out(1:strfind(out,10)-1);
end
end

function [ok, where, version] = getMake
[status,out]=system('where make');
ok = false;
where = '';
version = '';
if status ~= 0
    return;
end
lines=strfind(out,10);
where = out(1:lines(1)-1);
[status,out]=system('make -v');
if status ~= 0
    ok = false;
else
    ok = true;
end
if ok
    version = out(1:strfind(out,10)-1);
end
end

function [ok, where, version] = getMbed
[status,out]=system('where mbed');
ok = false;
where = '';
version = '';
if status ~= 0
    return;
end
lines=strfind(out,10);
where = out(1:lines(1)-1);
[status,out]=system('mbed --version');
if status ~= 0
    ok = false;
else
    ok = true;
end
if ok
    version = out(1:strfind(out,10)-1);
end
end

function [ok, where, version] = getMbedls
[status,out]=system('where mbedls');
ok = false;
where = '';
version = '';
if status ~= 0
    return;
end
lines=strfind(out,10);
where = out(1:lines(1)-1);
[status,out]=system('mbedls --version');
if status ~= 0
    ok = false;
else
    ok = true;
end
if ok
    version = out(1:strfind(out,10)-1);
end
end
