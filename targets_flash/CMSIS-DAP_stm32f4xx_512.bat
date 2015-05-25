REM %1 bin file
REM %2 root folder of mbed target
REM %3 mbed virtual drive
REM %4 COM port

"C:/CooCox/CoIDE/bin\coflash.exe" program STM32F401RB "%1" --adapter-name=CMSIS-DAP --port=SWD --adapter-clk=1000000 --erase=affected --reset=SYSRESETREQ --driver="C:/CooCox/CoIDE/flash/stm32f4xx_512.elf" 