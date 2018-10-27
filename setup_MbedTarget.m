%  MbedTarget Simulink target
%  Copyright (c) 2014-2018 Dr.O.Hagendorf , HS Wismar
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

 
function setup_MbedTarget

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
mbed.Prefs.setMbedDrive('D:');
mbed.Prefs.setMbedDownload('on');
mbed.Prefs.setMbedRTOS('off');
mbed.Prefs.setMbedProgrammer('mbed');


%% set user specific variables:
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
    [a,b,c]=mbed_getTargets();
    if(c>0)
        disp(' ');
        disp('supported mbed os 5 boards:')
        disp(b);
        disp(['found ' int2str(c) ' boards with "project.py -S targets"']);
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
