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

function [ MbedDrive, ComPort, MbedTarget5 ] = mbed_mbedls( )

MbedDrive = '';
ComPort = '';
MbedTarget5 = '';

[status,result]=system('mbedls -j');
if status==0 && size(result,2) > 3
    c=loadjson(result);
    if size(c,2)==1
        MbedDrive = c{1}.mount_point;
        ComPort = [c{1}.serial_port ':'];
        MbedTarget5 = c{1}.platform_name;
    end
end

end

