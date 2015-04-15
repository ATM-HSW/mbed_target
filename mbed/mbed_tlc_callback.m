function mbed_tlc_callback(hDlg, hSrc, paramName)
%Callback to get/set mbed target properties


if isequal(paramName, 'MbedPath')
    mbed.Prefs.setMbedPath(slConfigUIGetVal(hDlg, hSrc, paramName));
end

if isequal(paramName, 'MbedDrive')
    mbed.Prefs.setMbedDrive(slConfigUIGetVal(hDlg, hSrc, paramName));
end

if isequal(paramName, 'DownloadApplicationmbed_tlc_callback')
    mbed.Prefs.setMbedDownload(slConfigUIGetVal(hDlg, hSrc, paramName));
end

if isequal(paramName, 'MbedTarget')
    mbed.Prefs.setMbedTarget(slConfigUIGetVal(hDlg, hSrc, paramName));
end

if isequal(paramName, 'UseMbedRTOS')
    mbed.Prefs.setMbedRTOS(slConfigUIGetVal(hDlg, hSrc, paramName));
end

if isequal(paramName, 'ComPort')
    mbed.Prefs.getComPort(slConfigUIGetVal(hDlg, hSrc, paramName));
end
