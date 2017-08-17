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

