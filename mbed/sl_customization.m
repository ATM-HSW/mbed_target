function sl_customization(cm)
% SL_CUSTOMIZATION for mbed PIL connectivity config

% Copyright 2008-2014 The MathWorks, Inc.
%           2014 Dr. Olaf Hagendorf, HS Wismar

% disp('Mbed sl_customization called');

cm.registerTargetInfo(@loc_createSerialConfig);
cm.ExtModeTransports.add('mbed.tlc', 'tcpip',  'ext_comm', 'Level1');
cm.ExtModeTransports.add('mbed.tlc', 'serial', 'ext_serial_win32_comm', 'Level1');

% local function
function config = loc_createSerialConfig

config = rtw.connectivity.ConfigRegistry;
config.ConfigName = 'Mbed connectivity config using serial';
config.ConfigClass = 'mbed.ConnectivityConfig';

% matching system target file
config.SystemTargetFile = {'mbed.tlc'};

% match template makefile
config.TemplateMakefile = {'mbed.tmf'};

% match any hardware implementation
config.TargetHWDeviceType = {};
