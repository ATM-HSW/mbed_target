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

function sl_customization(cm)
% disp('Mbed sl_customization called');

cm.registerTargetInfo(@loc_createSerialConfig);
cm.ExtModeTransports.add('mbed_ert.tlc', 'tcpip',  'ext_comm', 'Level1');
cm.ExtModeTransports.add('mbed_ert.tlc', 'serial', 'ext_serial_win32_comm', 'Level1');

% local function
function config = loc_createSerialConfig

config = rtw.connectivity.ConfigRegistry;
config.ConfigName = 'Mbed connectivity config using serial';
config.ConfigClass = 'mbed.ConnectivityConfig';

% matching system target file
config.SystemTargetFile = {'mbed_ert.tlc'};

% match template makefile
config.TemplateMakefile = {'mbed_ert.tmf'};

% match any hardware implementation
config.TargetHWDeviceType = {};
