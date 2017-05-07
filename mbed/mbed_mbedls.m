function [ MbedDrive, ComPort, MbedTarget5 ] = mbed_mbedls( )

[status,result]=system('mbedls -j');
if status==0 && size(result,2) > 3
    c=loadjson(result);
    MbedDrive = c{1}.mount_point;
    ComPort = [c{1}.serial_port ':'];
    MbedTarget5 = c{1}.platform_name;
end

end

