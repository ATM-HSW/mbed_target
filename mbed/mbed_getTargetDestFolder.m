function [ path ] = mbed_getTargetDestFolder( mbedversion )

[~,modelName,~] = fileparts( which (bdroot));
if isequal(mbedversion, 'mbed-os 5')
    pathstr = mbed_getTargetRootPath();
    target = get_param(bdroot,'MbedTarget5');
    path = fullfile(pathstr,'targets', [modelName '_' target '_slprj'], '');
else
    pathstr = Simulink.fileGenControl('getConfig').CodeGenFolder;
    path = fullfile(pathstr, [modelName '_slprj']);
end

end

