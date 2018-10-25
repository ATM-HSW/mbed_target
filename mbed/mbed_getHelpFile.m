%  MbedTarget Simulink target
%  Copyright (c) 2014-2017 Dr.O.Hagendorf , HS Wismar
%
%  Licensed under the Apache License, Version 2.0 (the "License");
%  you may not use this file except in compliance with the License.
%  You may obtain a copy of the License at
%
%      http://www.apache.org/licenses/LICENSE-2.0
%
%  Unless required by applicable law or agreed to in writing, software
%  distributed under the License is distributed on an "AS IS" BASIS,
%  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
%  See the License for the specific language governing permissions and
%  limitations under the License.

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
  addStruct('digitalOutput',            'gpio/Digital_Output_en');
  addStruct('digitalOutputLED',         'gpio/Digital_Output_LED_en');
  addStruct('digitalInput',             'gpio/Digital_Input_en');
  addStruct('digitalInputUserButton',   'gpio/Digital_Input_UserButton_en');
  addStruct('Interrupt',                'gpio/Interrupt_en');
  addStruct('analogOutput',             'gpio/Analog_Output_en');
  addStruct('analogInput',              'gpio/Analog_Input_en');
  addStruct('PWMOutput',                'gpio/PWM_Output_en');
  addStruct('timerServoOutput',         'gpio/Servo_Output_en');
  addStruct('RuntimeDAC',               'gpio/RuntimeDac_en');
  addStruct('RuntimeGPIO',              'gpio/RuntimeGpio_en');

  addStruct('RTOScreateThread',         'rtos/Thread_en');
  addStruct('RTOScreateTimer',          'rtos/Timer_en');
  addStruct('RTOSmailSend',             'rtos/Mail_Send_en');
  addStruct('RTOSmailReceive',          'rtos/Mail_Receive_en');
  addStruct('RTOSThreadwait',           'rtos/Threadwait_en');

  addStruct('Max11300_Config',          'converter_extender/Max11300_Config_en');
  addStruct('Max11300_digitalInput',    'converter_extender/Max11300_Digital_Input_en');
  addStruct('Max11300_digitalOutput',   'converter_extender/Max11300_Digital_Output_en');
  addStruct('Max11300_analogInput',     'converter_extender/Max11300_Analog_Input_en');
  addStruct('Max11300_analogOutput',    'converter_extender/Max11300_Analog_Output_en');
  addStruct('Max11300_temperature',     'converter_extender/Max11300_Temperature_en');
  addStruct('mcp3428',                  'converter_extender/Mcp3428_Analog_Input_en');
  addStruct('mcp4728',                  'converter_extender/Mcp4728_Analog_Output_en');
  addStruct('PCF8574DigitalOutput',     'converter_extender/Pcf8574_Digital_Output_en');
  addStruct('PCF8574DigitalInput',      'converter_extender/Pcf8574_Digital_Input_en');
  addStruct('mcp4822',                  'converter_extender/Mcp4822_Analog_Output_en');
  addStruct('mcp3204',                  'converter_extender/Mcp3204_Analog_Input_en');

  addStruct('tmp102',                   'devices/TMP102_en');
  addStruct('lm75b',                    'devices/LM75B_en');
  addStruct('bmp180',                   'devices/BMP180_en');
  addStruct('vl6180x',                  'devices/VL6180X_en');
  addStruct('lsm303dlhc',               'devices/LSM303DLHC_en');
  addStruct('mpu9250',                  'devices/MPU9250_en');
  addStruct('mpu9250_ak8963',           'devices/MPU9250_AK8963_en');
  addStruct('tmp123',                   'devices/TMP123_en');
  addStruct('lis302dl',                 'devices/LIS302DL_en');
  addStruct('l3gd20',                   'devices/L3GD20_en');
  addStruct('max6675',                  'devices/MAX6675_en');
  addStruct('ds1820',                   'devices/DS1820_en');
  addStruct('DisplayCR12832',           'devices/DisplayCR12832_en');
  addStruct('DS3231',                   'devices/DS3231_en');

  addStruct('fm24cl16bRead',            'memory/Fm24cl16b_Fram_Read_en');
  addStruct('fm24cl16bWrite',           'memory/Fm24cl16b_Fram_Write_en');
  addStruct('fm25l16bRead',             'memory/Fm25l16b_Fram_Read_en');
  addStruct('fm25l16bWrite',            'memory/Fm25l16b_Fram_Write_en');

  addStruct('OneWireConfig',            'serial/one_wire_config_en');
  addStruct('OneWireReadROM',           'serial/one_wire_search_rom_en');
  addStruct('TLC5952Config',            'serial/TLC5952Config_en');
  addStruct('TLC5962Ventil',            'serial/TLC5962Ventil_en');
  addStruct('TLC5952Balken',            'serial/TLC5952Balken_en');
  addStruct('SerialUARTConfig',         'serial/UART_Config_en');
  addStruct('SerialUARTConfigUSB',      'serial/UART_ConfigUSB_en');
  addStruct('SerialUARTWrite',          'serial/UART_Write_en');
  addStruct('SerialUARTWriteBlock',     'serial/UART_Write_Block_en');
  addStruct('SerialUARTRead',           'serial/UART_Read_en');
  addStruct('SerialUARTReadBlock',      'serial/UART_Read_Block_en');
  addStruct('SerialUARTReadLine',       'serial/UART_Read_Line_en');
  addStruct('SerialUSBCDCConfig',       'serial/USB_CDC_Config_en');
  addStruct('SerialUSBCDCWrite',        'serial/USB_CDC_Write_Block_en');
  addStruct('SerialUSBCDCRead',         'serial/USB_CDC_Read_en');
  addStruct('I2CConfig',                'serial/I2c_Config_en');
  addStruct('I2CWrite',                 'serial/I2c_Write_en');
  addStruct('I2CRead',                  'serial/I2c_Read_en');
  addStruct('SPIMasterConfig',          'serial/Spi_Master_Config_en');
  addStruct('SPISlaveConfig',           'serial/Spi_Slave_Config_en');
  addStruct('SPIMasterReadWriteConfig', 'serial/Spi_Master_Read_Write_en');
  addStruct('SPISlaveReadConfig',       'serial/Spi_Slave_Read_en');
  addStruct('SPISlaveWriteConfig',      'serial/Spi_Slave_Write_en');
  addStruct('SerialCANConfig',          'serial/CAN_Config_en');
  addStruct('SerialCANReceive',         'serial/CAN_Receive_en');
  addStruct('SerialCANSend',            'serial/CAN_Send_en');

  addStruct('CSVWriter',                'several/CsvWriter_en');
  addStruct('CSVReader',                'several/CsvReader_en');

  addStruct('randomNumber',             'targets/target_specific/Random_Number_Generator_en');
  addStruct('timerEncoderInput',        'targets/target_specific/Encoder_Input_en');
  addStruct('timerInputCapture',        'targets/target_specific/InputCapture_en');
  addStruct('timerCounter',             'targets/target_specific/Counter_en');

  addStruct('EthernetIPConfigStack',    'ethernet/EthernetIPConfigStack_en');
  addStruct('UDPConfigSocket',          'ethernet/UDPConfigSocket_en');
  %addStruct('UDPServerWrite',           'ethernet/UDPServerWrite_en');
  %addStruct('UDPServerWriteCSV',        'ethernet/UDPServerWriteCSV_en');
  %addStruct('UDPServerRead',            'ethernet/UDPServerRead_en');
  %addStruct('UDPServerReadCSV',         'ethernet/UDPServerReadCSV_en');
  addStruct('UDPClientSendBlock',       'ethernet/UDPClientSendBlock_en');
  addStruct('UDPClientSend',            'ethernet/UDPClientSend_en');
  addStruct('UDPClientReceiveBlock',    'ethernet/UDPClientReceiveBlock_en');
  addStruct('UDPClientRead',            'ethernet/UDPClientReceive_en');
  addStruct('MQTTConfig',               'ethernet/MQTTConfig_en');
  addStruct('MQTTPublish',              'ethernet/MQTTPub_en');
  addStruct('MQTTSubscribe',            'ethernet/MQTTSub_en');
  addStruct('CoAPConfigStack',          'ethernet/COAPConfigStack_en');
  addStruct('CoAPGet',                  'ethernet/COAPRequestGet_en');
  addStruct('CoAPPut',                  'ethernet/COAPRequestPut_en');
%   addStruct('', '');
%   addStruct('', '');
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