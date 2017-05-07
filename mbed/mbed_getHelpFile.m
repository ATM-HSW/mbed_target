function path = mbed_getHelpFile(blockname)
  global cellBlocks;
  global cellFiles;
  if isempty(cellBlocks)
      createStruct();
  end
  res = strcmp(cellBlocks, blockname);
  idx = find(res, 1);
  if isempty(idx)
      path  = fullfile(mbed_getTargetRootPath(), 'doc/Error404.html');
  else
      path = fullfile(mbed_getTargetRootPath(), 'htmlhelp', [cellFiles{idx}, '.html']); 
  end
end

function createStruct()
  global cellBlocks;
  global cellFiles;
  cellBlocks = {};
  cellFiles = {};
  addStruct('digitalOutput', 'mbed/gpio_Digital_Output_en');
  addStruct('digitalOutputLED', 'mbed/gpio_Digital_Output_LED_en');
  addStruct('digitalInput', 'mbed/gpio_Digital_Input_en');
  addStruct('digitalInputUserButton', 'mbed/gpio_Digital_Input_UserButton_en');
  addStruct('Interrupt', 'mbed/gpio_Interrupt_en');
  addStruct('analogOutput', 'mbed/gpio_Analog_Output_en');
  addStruct('analogInput', 'mbed/gpio_Analog_Input_en');
  addStruct('PWMOutput', 'mbed/gpio_PWM_Output_en');
  addStruct('Servo', 'mbed/gpio_Servo_Output_en');
  addStruct('RTOScreateThread', 'mbed/rtos_Thread_en');
  addStruct('RTOScreateTimer', 'mbed/rtos_Timer_en');
  addStruct('RTOSmailSend', 'mbed/rtos_Mail_Send_en');
  addStruct('RTOSmailReceive', 'mbed/rtos_Mail_Receive_en');
  addStruct('RTOSThreadwait', 'mbed/rtos_Threadwait_en');
  addStruct('SerialUARTConfig', 'mbed/serial_UART_Config_en');
  addStruct('SerialUARTConfigUSB', 'mbed/serial_UART_ConfigUSB_en');
  addStruct('SerialUARTWriteBlock', 'mbed/serial_UART_Write_Block_en');
  addStruct('SerialUARTRead', 'mbed/serial_UART_Read_en');
  addStruct('SerialUSBCDCConfig', 'mbed/serial_USB_CDC_Config_en');
  addStruct('SerialUSBCDCWrite', 'mbed/serial_USB_CDC_Write_Block_en');
  addStruct('SerialUSBCDCRead', 'mbed/serial_USB_CDC_Read_en');
  addStruct('I2CConfig', 'mbed/serial_I2c_Config_en');
  addStruct('I2CWrite', 'mbed/serial_I2c_Write_en');
  addStruct('I2CRead', 'mbed/serial_I2c_Read_en');
  addStruct('SPIMasterConfig', 'mbed/serial_Spi_Master_Config_en');
  addStruct('SPISlaveConfig', 'mbed/serial_Spi_Slave_Config_en');
  addStruct('SPIMasterReadWriteConfig', 'mbed/serial_Spi_Master_Read_Write_en');
  addStruct('SPISlaveReadConfig', 'mbed/serial_Spi_Slave_Read_en');
  addStruct('SPISlaveWriteConfig', 'mbed/serial_Spi_Slave_Write_en');
  addStruct('SerialCANConfig', 'mbed/serial_CAN_Config_en');
  addStruct('SerialCANReceive', 'mbed/serial_CAN_Receive_en');
  addStruct('SerialCANSend', 'mbed/serial_CAN_Send_en');
%   addStruct('', '');
%   addStruct('', '');
%   addStruct('', '');
%   addStruct('', '');
end

function addStruct(blockname, htmlfile)
  global cellBlocks;
  global cellFiles;
  cellBlocks = [cellBlocks, {blockname}];
  cellFiles = [cellFiles, {htmlfile}];
end