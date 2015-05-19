REM %1 bin file
REM %2 root folder of mbed target
REM %3 mbed virtual drive
REM %4 COM port

python.exe %2\targets_flash\stm32loader_reset2bootloader.py
python.exe %2\targets_flash\stm32loader.py -p %4 -e -w -g 0x08000000 %1 0x08000000 -Rst -Run -Q -V after_programming
