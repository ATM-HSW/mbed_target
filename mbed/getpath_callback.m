function getpath_callback(hDlg, hSrc, paramName)
%Callback to get/set mbed target properties


if isequal(paramName, 'MbedPath')
    mbed.Prefs.setMbedPath(slConfigUIGetVal(hDlg, hSrc, paramName));
end

if isequal(paramName, 'MbedDrive')
    mbed.Prefs.setMbedDrive(slConfigUIGetVal(hDlg, hSrc, paramName));
end