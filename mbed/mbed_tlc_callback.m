function mbed_tlc_callback(hDlg, hSrc, paramName)
%Callback to get/set mbed target properties

if isequal(paramName, 'MbedVersion')
    mbedversion = (slConfigUIGetVal(hDlg, hSrc, paramName));
    mbed.Prefs.setMbedVersion(mbedversion);
    if isequal(mbedversion, 'mbed-os 5')
        slConfigUISetEnabled(hDlg, hSrc,'MbedTarget',0);
        slConfigUISetEnabled(hDlg, hSrc,'MbedTarget5',1);
        slConfigUISetEnabled(hDlg, hSrc,'UseMbedRTOS',0);
        slConfigUISetVal(hDlg, hSrc, 'UseMbedRTOS', 1);
    else
        slConfigUISetEnabled(hDlg, hSrc,'MbedTarget',1);
        slConfigUISetEnabled(hDlg, hSrc,'MbedTarget5',0);
        slConfigUISetEnabled(hDlg, hSrc,'UseMbedRTOS',1);
    end
end

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

if isequal(paramName, 'MbedTarget')
    mbed.Prefs.setMbedTarget(slConfigUIGetVal(hDlg, hSrc, paramName));
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

