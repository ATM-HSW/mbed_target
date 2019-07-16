% Inhalt Combobox mit allen mbed Targets
param = get_param(get_param(0,'CurrentSystem'),'ObjectParameters');
liste = param.MbedTarget5.Enum;

% aktuelles target aus Modell Config
target = get_param(get_param(0,'CurrentSystem'),'MbedTarget5');


maskObj = Simulink.Mask.get(gcb);
maskObj.Parameters(1,1).TypeOptions={'a','b'}

[ret1, ret2, count]= mbed_getTargets();
f=strfind(ret1,'|');

maskObj.Parameters(1,1).TypeOptions={ret1(1:f(1)-1)};
maskObj.Parameters(1,1).TypeOptions={maskObj.Parameters(1,1).TypeOptions{:}, ret1(f(1)+1:f(2)-1)};
maskObj.Parameters(1,1).TypeOptions={maskObj.Parameters(1,1).TypeOptions{:}, ret1(f(2)+1:f(3)-1)};