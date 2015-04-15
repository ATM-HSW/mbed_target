function mbed_tlc_opencallback()
    disp 'mbed_tlc_opencallback called';
    try
        get_param(bdroot,'MbedPath');
    catch
        add_param(bdroot,'MbedPath', mbed.Prefs.getMbedPath());
    end
end
