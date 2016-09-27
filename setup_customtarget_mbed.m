function setup_customtarget_mbed

addpath(fullfile(pwd,'mbed'),fullfile(pwd,'blocks'),fullfile(pwd,'blocks','mex'),fullfile(pwd,'blocks','slx'));

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

