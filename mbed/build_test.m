target = 'NUCLEO_F446RE';
jobs = 4;
mbedappcfg = 'MbedMinimal';

folder = '\examples\Applications_Boards_etc\AppShield_CR12832_LM75B';
% model = 'nucleo_f446_AppShield';
% [status, time] = testbuild(folder, model, target, jobs, mbedappcfg);
% disp([model ': ' status ' : ' time]);

% model = 'nucleo_f446_lm75b';
% [status, time] = testbuild(folder, model, target, jobs, mbedappcfg);
% disp([model ': ' status ' : ' time]);

model = 'DreiPunktregler';
[status, time] = testbuild(folder, model, target, jobs, mbedappcfg);
disp([model ': ' status ' : ' time]);

% folder = '\examples\general';
% model = 'blinky';
% [status, time] = testbuild(folder, model, target, jobs, mbedappcfg);
% disp([model ': ' status ' : ' time]);
% 
% model = 'pushbutton';
% [status, time] = testbuild(folder, model, target, jobs, mbedappcfg);
% disp([model ': ' status ' : ' time]);
% 
% model = 'Interrupt_push_button';
% [status, time] = testbuild(folder, model, target, jobs, mbedappcfg);
% disp([model ': ' status ' : ' time]);



function [status, time] = testbuild(folder, modelname, target, jobs, mbedappcfg)
root = mbed_getTargetRootPath();
folder = [root folder];
cd(folder);

set_param(modelname,'MbedTarget5', target)
set_param(modelname,'MakeJobs', num2str(jobs))
set_param(modelname,'MbedAppConfig', mbedappcfg)
set_param(modelname,'MbedlsAutodetect', 0)
set_param(modelname,'DownloadApplication', 0)
buildinfo = evalc(['rtwbuild(''' modelname ''')']);
ret = contains(buildinfo,'### Successful completion of build procedure for model:');
if ret, status='OK'; else, status='Error'; end
begin= strfind(buildinfo,'Elapsed time is ');
ende = strfind(buildinfo(begin:end),' seconds.');
time = buildinfo(begin:begin+ende+7);
end
