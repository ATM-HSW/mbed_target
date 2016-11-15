function mbed_tlc_callback(hDlg, hSrc, paramName)
%Callback to get/set mbed target properties

if isequal(paramName, 'MbedVersion')
    mbedversion = (slConfigUIGetVal(hDlg, hSrc, paramName));
    if isequal(mbedversion, 'mbed-os 5')
      slConfigUISetEnabled(hDlg, hSrc,'MbedTarget',0);
      slConfigUISetEnabled(hDlg, hSrc,'MbedTarget5',1);
      slConfigUISetVal(hDlg, hSrc, 'UseMbedRTOS', 1);
      slConfigUISetEnabled(hDlg, hSrc,'UseMbedRTOS',0);
    else
      slConfigUISetEnabled(hDlg, hSrc,'MbedTarget',1);
      slConfigUISetEnabled(hDlg, hSrc,'MbedTarget5',0);
      slConfigUISetEnabled(hDlg, hSrc,'UseMbedRTOS',1);
    end
end

if isequal(paramName, 'mbedls')
    [a,b]=system('mbedls -j');
    if a==0 && size(b,2) > 3
        c=loadjson(b);
        slConfigUISetVal(hDlg, hSrc, 'MbedDrive', c{1}.mount_point);
        slConfigUISetVal(hDlg, hSrc, 'ComPort', [c{1}.serial_port ':']);
        slConfigUISetVal(hDlg, hSrc, 'MbedTarget5', c{1}.platform_name);
    end
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

if isequal(paramName, 'MbedDownloadApp')
    % mbed.Prefs.setMbedRTOS(slConfigUIGetVal(hDlg, hSrc, paramName));
end

