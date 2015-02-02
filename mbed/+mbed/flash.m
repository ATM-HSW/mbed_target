function flash( modelName )
%FLASH - target flashing
%   Copyright 2008-2014 The MathWorks, Inc.

rtw = RTW.GetBuildDir(modelName);
file = fullfile(rtw.BuildDirectory, [ modelName, '.hex']);

if exist(file, 'file')
    mbed.runAvrDude(file);
else
    msgbox(['File ', file ' not found'], 'Target flasing', 'error');
end

end

