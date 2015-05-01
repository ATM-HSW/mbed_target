REM %1 bin file
REM %2 root folder of mbed target
REM %3 mbed virtual drive
REM %4 COM port

"C:\Program Files (x86)\STMicroelectronics\STM32 ST-LINK Utility\ST-LINK Utility\ST-LINK_CLI" -c SWD -P %1 0x08000000 -Rst -Run -Q -V after_programming