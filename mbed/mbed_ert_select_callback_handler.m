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

function mbed_select_callback_handler(hDlg, hSrc)
  % The target is model reference compliant
  slConfigUISetVal(hDlg, hSrc, 'ModelReferenceCompliant', 'on');
  slConfigUISetEnabled(hDlg, hSrc, 'ModelReferenceCompliant', false);

  % Hardware being used is the production hardware
  slConfigUISetVal(hDlg, hSrc, 'ProdEqTarget', 'on');

  % Setup C++ as default language
  slConfigUISetVal(hDlg, hSrc, 'TargetLang', 'C++');

  % Setup the hardware configuration
  slConfigUISetVal(hDlg, hSrc, 'ProdHWDeviceType', 'ARM Compatible->ARM Cortex');

  % Set the TargetLibSuffix
  slConfigUISetVal(hDlg, hSrc, 'TargetLibSuffix', '.a');

  % For real-time builds, we must generate ert_main.c
  slConfigUISetVal(hDlg, hSrc, 'ERTCustomFileTemplate', 'mbed_ert_file_process.tlc');
  slConfigUISetVal(hDlg, hSrc, 'CustomSource', '');

  % switch on dialog to be able to configure custom step/init function names
  slConfigUISetVal(hDlg,hSrc,'ModelStepFunctionPrototypeControlCompliant','on');
  slConfigUISetEnabled(hDlg,hSrc,'ModelStepFunctionPrototypeControlCompliant',false);


  slConfigUISetVal(hDlg,hSrc,'GenerateReport','off');

%  slConfigUISetEnabled(hDlg, hSrc,'UseMbedRTOS',0);
%  slConfigUISetVal(hDlg, hSrc, 'UseMbedRTOS', 1);

%  slConfigUISetVal(hDlg, hSrc, 'GenCodeOnly', 'on');

  % set default solver config
  slConfigUISetVal(hDlg, hSrc, 'SolverType', 'Fixed-step');
  slConfigUISetVal(hDlg, hSrc, 'Solver', 'FixedStepDiscrete');
  slConfigUISetVal(hDlg, hSrc, 'FixedStep', '1e-3');
  slConfigUISetVal(hDlg, hSrc, 'SolverMode', 'SingleTasking');
end
