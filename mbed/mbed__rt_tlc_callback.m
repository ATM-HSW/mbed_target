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

function mbed__rt_tlc_callback(hDlg, hSrc, paramName)

if isequal(paramName, 'mbedls')
    [ MbedDrive, ComPort, MbedTarget5 ] = mbed_mbedls();
    if strlength(MbedDrive) > 1
        slConfigUISetVal(hDlg, hSrc, 'DownloadMethod', 'mbed');
        slConfigUISetVal(hDlg, hSrc, 'MbedDrive', MbedDrive);
        slConfigUISetVal(hDlg, hSrc, 'ComPort', ComPort);
        slConfigUISetVal(hDlg, hSrc, 'MbedTarget5', MbedTarget5);
    end
end

if isequal(paramName, 'DownloadApplication')
    mbed.Prefs.setMbedDownload(slConfigUIGetVal(hDlg, hSrc, paramName));
end

if isequal(paramName, 'MbedTarget5')
    mbed.Prefs.setMbedTarget5(slConfigUIGetVal(hDlg, hSrc, paramName));
end

if isequal(paramName, 'MbedlsAutodetect')
    mbed.Prefs.setMbedAutodetect(slConfigUIGetVal(hDlg, hSrc, paramName));
end

if isequal(paramName, 'UseMbedRTOS')
    mbed.Prefs.setMbedRTOS(slConfigUIGetVal(hDlg, hSrc, paramName));
end

if isequal(paramName, 'ComPort')
    mbed.Prefs.getComPort(slConfigUIGetVal(hDlg, hSrc, paramName));
end

if isequal(paramName, 'MbedDownloadApp')
    % mbed.Prefs.setMbedRTOS(slConfigUIGetVal(hDlg, hSrc, paramName));
end

