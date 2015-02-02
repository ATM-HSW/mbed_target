%MBED_SELECT_CALLBACK_HANDLER callback handler for mbed target

%   Copyright 2009-2014 The MathWorks, Inc.
%             2014 Dr. Olaf Hagendorf, HS Wismar

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
	slConfigUISetVal(hDlg, hSrc, 'ERTCustomFileTemplate', 'mbed_file_process.tlc');
	
	slConfigUISetVal(hDlg, hSrc, 'GenCodeOnly', 'on');
	
	slConfigUISetVal(hDlg, hSrc, 'SolverType', 'Fixed-step');
	slConfigUISetVal(hDlg, hSrc, 'Solver', 'FixedStepDiscrete');
	slConfigUISetVal(hDlg, hSrc, 'FixedStep', '1e-3');
	slConfigUISetVal(hDlg, hSrc, 'SolverMode', 'SingleTasking');
end
