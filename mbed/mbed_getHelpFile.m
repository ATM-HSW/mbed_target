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
      path = fullfile(mbed_getTargetRootPath(), 'doc/_build/htmlhelp', [cellFiles{idx}, '.html']); 
  end
end

function createStruct()
  global cellBlocks;
  global cellFiles;
  cellBlocks = {};
  cellFiles = {};
  addStruct('digitalOutputLED', 'mbed/gpio_Digital_Output_LED_en');
  addStruct('digitalInputUserButton', 'mbed/gpio_Digital_Input_UserButton_en');
  addStruct('digitalOutput', 'mbed/gpio_Digital_Output_en');
  addStruct('digitalInput', 'mbed/gpio_Digital_Input_en');
  addStruct('Interrupt', 'mbed/gpio_Interrupt_en');
  addStruct('PWMOutput', 'mbed/gpio_PWM_Output_en');
  addStruct('analogOutput', 'mbed/gpio_Analog_Output_en');
  addStruct('analogInput', 'mbed/gpio_Analog_Input_en');
  addStruct('Servo', 'mbed/gpio_Servo_Output_en');
%   addStruct('', '');
end

function addStruct(blockname, htmlfile)
  global cellBlocks;
  global cellFiles;
  cellBlocks = {cellBlocks{:}, blockname};
  cellFiles = {cellFiles{:}, htmlfile};
end