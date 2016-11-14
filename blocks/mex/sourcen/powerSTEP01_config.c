/* Copyright 2010 The MathWorks, Inc. */
/* Copyright 2016 Sven Lack */
/*
 *   sfunar_serialConfig.c Simple C-MEX S-function for function call.
 *
 *   ABSTRACT:
 *     The purpose of this SFunction is to call a simple legacy
 *     function during simulation:
 *
 *        serialConfigOutput()
 *
 *   Simulink version           : 7.3 (R2009a) 15-Jan-2009
 *   C source code generated on : 30-Jun-2009 18:46:58
 */

/*
 * Must specify the S_FUNCTION_NAME as the name of the S-function.
 */
#define S_FUNCTION_NAME                powerSTEP01_config
#define S_FUNCTION_LEVEL               2

/*
 * Need to include simstruc.h for the definition of the SimStruct and
 * its associated macro definitions.
 */
#include "simstruc.h"
#define EDIT_OK(S, P_IDX) \
 (!((ssGetSimMode(S)==SS_SIMMODE_SIZES_CALL_ONLY) && mxIsEmpty(ssGetSFcnParam(S, P_IDX))))
#define SAMPLE_TIME                    (ssGetSFcnParam(S, 0))
 
 /*
 * Utility function prototypes.
 */
static bool IsRealMatrix(const mxArray * const m);

#define MDL_CHECK_PARAMETERS
#if defined(MDL_CHECK_PARAMETERS) && defined(MATLAB_MEX_FILE)

/* Function: mdlCheckParameters ===========================================
 * Abstract:
 *    mdlCheckParameters verifies new parameter settings whenever parameter
 *    change or are re-evaluated during a simulation. When a simulation is
 *    running, changes to S-function parameters can occur at any time during
 *    the simulation loop.
 */
static void mdlCheckParameters(SimStruct *S)
{
	
   // /*
   // * Check the parameter 1 
   // */
  // if EDIT_OK(S, 0) {
    // int_T dimsArray[2] = { 1, 1 };

    // /* Check the parameter attributes */
    // ssCheckSFcnParamValueAttribs(S, 0, "P1", DYNAMICALLY_TYPED, 2, dimsArray, 0);
  // }
   // /*
   // * Check the parameter 2 ACC
   // */
  // if EDIT_OK(S, 1) {
    // int_T dimsArray[2] = { 1, 1 };

    // /* Check the parameter attributes */
    // ssCheckSFcnParamValueAttribs(S, 1, "ACC", DYNAMICALLY_TYPED, 2, dimsArray, 0);
  // }
  
      // /*
   // * Check the parameter 3 DEC
   // */
  // if EDIT_OK(S, 2) {
    // int_T dimsArray[2] = { 1, 1 };

    // /* Check the parameter attributes */
    // ssCheckSFcnParamValueAttribs(S, 2, "DEC", DYNAMICALLY_TYPED, 2, dimsArray, 0);
  // }
  
   // /*
   // * Check the parameter 4 Max Speed
   // */
  // if EDIT_OK(S, 3) {
    // int_T dimsArray[2] = { 1, 1 };

    // /* Check the parameter attributes */
    // ssCheckSFcnParamValueAttribs(S, 3, "Max_Speed", DYNAMICALLY_TYPED, 2, dimsArray, 0);
  // }
  
     // /*
   // * Check the parameter 5 Min Speed
   // */
  // if EDIT_OK(S, 4) {
    // int_T dimsArray[2] = { 1, 1 };

    // /* Check the parameter attributes */
    // ssCheckSFcnParamValueAttribs(S, 4, "Min_Speed", DYNAMICALLY_TYPED, 2, dimsArray, 0);
  // }
  
   // /*
   // * Check the parameter 6 Full Step Speed
   // */
  // if EDIT_OK(S, 5) {
    // int_T dimsArray[2] = { 1, 1 };

    // /* Check the parameter attributes */
    // ssCheckSFcnParamValueAttribs(S, 5, "Full_Step_Speed", DYNAMICALLY_TYPED, 2, dimsArray, 0);
  // } 
   
   // /*
   // * Check the parameter 7 Overcurrent
   // */
  // if EDIT_OK(S, 6) {
    // int_T dimsArray[2] = { 1, 1 };

    // /* Check the parameter attributes */
    // ssCheckSFcnParamValueAttribs(S, 6, "Overcurrent", DYNAMICALLY_TYPED, 2, dimsArray, 0);
  // } 
   
   // /*
   // * Check the parameter 8 Step_Mode
   // */
  // if EDIT_OK(S, 7) {
    // int_T dimsArray[2] = { 1, 1 };

    // /* Check the parameter attributes */
    // ssCheckSFcnParamValueAttribs(S, 7, "Step_Mode", DYNAMICALLY_TYPED, 2, dimsArray, 0);
  // } 
  
     // /*
   // * Check the parameter 9 Hold_torque
   // */
  // if EDIT_OK(S, 8) {
    // int_T dimsArray[2] = { 1, 1 };

    // /* Check the parameter attributes */
    // ssCheckSFcnParamValueAttribs(S, 8, "Hold_torque", DYNAMICALLY_TYPED, 2, dimsArray, 0);
  // } 
  
       // /*
   // * Check the parameter 10 Run_torque
   // */
  // if EDIT_OK(S, 9) {
    // int_T dimsArray[2] = { 1, 1 };

    // /* Check the parameter attributes */
    // ssCheckSFcnParamValueAttribs(S, 9, "Run_torque", DYNAMICALLY_TYPED, 2, dimsArray, 0);
  // } 
  
    // /*
   // * Check the parameter 11 ACC_torque
   // */
  // if EDIT_OK(S, 10) {
    // int_T dimsArray[2] = { 1, 1 };

    // /* Check the parameter attributes */
    // ssCheckSFcnParamValueAttribs(S, 10, "ACC_torque", DYNAMICALLY_TYPED, 2, dimsArray, 0);
  // }
  
    // /*
   // * Check the parameter 12 DEC_torque
   // */
  // if EDIT_OK(S, 11) {
    // int_T dimsArray[2] = { 1, 1 };

    // /* Check the parameter attributes */
    // ssCheckSFcnParamValueAttribs(S, 11, "DEC_torque", DYNAMICALLY_TYPED, 2, dimsArray, 0);
  // }  
  
   // /*
   // * Check the parameter 13 SPI_Nr
   // */
  // if EDIT_OK(S, 12) {
    // int_T dimsArray[2] = { 1, 1 };

    // /* Check the parameter attributes */
    // ssCheckSFcnParamValueAttribs(S, 12, "SPI_Nr", DYNAMICALLY_TYPED, 2, dimsArray, 0);
  // }   
  
   // /*
   // * Check the parameter 14 Current_Voltage_Mode_SEL
   // */
  // if EDIT_OK(S, 13) {
    // int_T dimsArray[2] = { 1, 1 };

    // /* Check the parameter attributes */
    // ssCheckSFcnParamValueAttribs(S, 13, "Current_Voltage_Mode_SEL", SS_BOOLEAN, 2, dimsArray, 0);
  // }   
  
     // /*
   // * Check the parameter 15 Reset_Port
   // */
  // if EDIT_OK(S, 14) {
    // int_T dimsArray[2] = { 1, 1 };

    // /* Check the parameter attributes */
    // ssCheckSFcnParamValueAttribs(S, 14, "Reset_Port", DYNAMICALLY_TYPED, 2, dimsArray, 0);
  // }   
  
   // /*
   // * Check the parameter 16 Reset_Pin
   // */
  // if EDIT_OK(S, 15) {
    // int_T dimsArray[2] = { 1, 1 };

    // /* Check the parameter attributes */
    // ssCheckSFcnParamValueAttribs(S, 15, "Reset_Pin", DYNAMICALLY_TYPED, 2, dimsArray, 0);
  // }   
     // /*
   // * Check the parameter 17 SSEL_Port
   // */
  // if EDIT_OK(S, 16) {
    // int_T dimsArray[2] = { 1, 1 };

    // /* Check the parameter attributes */
    // ssCheckSFcnParamValueAttribs(S, 16, "SSEL_Port", DYNAMICALLY_TYPED, 2, dimsArray, 0);
  // }   
  
   // /*
   // * Check the parameter 18 SSEL_Pin
   // */
  // if EDIT_OK(S, 17) {
    // int_T dimsArray[2] = { 1, 1 };

    // /* Check the parameter attributes */
    // ssCheckSFcnParamValueAttribs(S, 17, "SSEL_Pin", DYNAMICALLY_TYPED, 2, dimsArray, 0);
  // }   
  

}

#endif

/* Function: mdlInitializeSizes ===========================================
 * Abstract:
 *    The sizes information is used by Simulink to determine the S-function
 *    block's characteristics (number of inputs, outputs, states, etc.).
 */
static void mdlInitializeSizes(SimStruct *S)
{
  /* Number of expected parameters */
  ssSetNumSFcnParams(S, 44);

#if defined(MATLAB_MEX_FILE)

  if (ssGetNumSFcnParams(S) == ssGetSFcnParamsCount(S)) {
    /*
     * If the number of expected input parameters is not equal
     * to the number of parameters entered in the dialog box return.
     * Simulink will generate an error indicating that there is a
     * parameter mismatch.
     */
    mdlCheckParameters(S);
    if (ssGetErrorStatus(S) != NULL) {
      return;
    }
  } else {
    /* Return if number of expected != number of actual parameters */
    return;
  }

#endif

  /* Set the parameter's tunable status */
  ssSetSFcnParamTunable(S, 0, 0);	// P1
  ssSetSFcnParamTunable(S, 1, 0);	// ACC
  ssSetSFcnParamTunable(S, 2, 0);	// DEC
  ssSetSFcnParamTunable(S, 3, 0);	// Max_Speed
  ssSetSFcnParamTunable(S, 4, 0);	// Min_Speed
  ssSetSFcnParamTunable(S, 5, 0);	// Full_Step_Speed
  ssSetSFcnParamTunable(S, 6, 0);	// Overcurrent
  ssSetSFcnParamTunable(S, 7, 0);	// Step_Mode
  ssSetSFcnParamTunable(S, 8, 0);	// Hold_torque
  ssSetSFcnParamTunable(S, 9, 0);	// Run_torque
  ssSetSFcnParamTunable(S, 10, 0);	// ACC_torque
  ssSetSFcnParamTunable(S, 11, 0);	// DEC_torque
  ssSetSFcnParamTunable(S, 12, 0);	// SPI_Nr
  ssSetSFcnParamTunable(S, 13, 0);	// Current_Voltage_Mode_SEL
  ssSetSFcnParamTunable(S, 14, 0);	// Reset_Port
  ssSetSFcnParamTunable(S, 15, 0);	// Reset_Pin
  ssSetSFcnParamTunable(S, 16, 0);	// SSEL_Port
  ssSetSFcnParamTunable(S, 17, 0);	// SSEL_Pin
  ssSetSFcnParamTunable(S, 18, 0);	// ACC_1
  ssSetSFcnParamTunable(S, 19, 0);	// DEC_1
  ssSetSFcnParamTunable(S, 20, 0);	// Max_Speed_1
  ssSetSFcnParamTunable(S, 21, 0);	// Min_Speed_1
  ssSetSFcnParamTunable(S, 22, 0);	// Full_Step_Speed_1
  ssSetSFcnParamTunable(S, 23, 0);	// Overcurrent_1
  ssSetSFcnParamTunable(S, 24, 0);	// Step_Mode_1
  ssSetSFcnParamTunable(S, 25, 0);	// Hold_torque_1
  ssSetSFcnParamTunable(S, 26, 0);	// Run_torque_1
  ssSetSFcnParamTunable(S, 27, 0);	// ACC_torque_1
  ssSetSFcnParamTunable(S, 28, 0);	// DEC_torque_1
  
  ssSetSFcnParamTunable(S, 29, 0);	// Current_Voltage_Mode_SEL_1

  ssSetSFcnParamTunable(S, 30, 0);	// ACC_2
  ssSetSFcnParamTunable(S, 31, 0);	// DEC_2
  ssSetSFcnParamTunable(S, 32, 0);	// Max_Speed_2
  ssSetSFcnParamTunable(S, 33, 0);	// Min_Speed_2
  ssSetSFcnParamTunable(S, 34, 0);	// Full_Step_Speed_2
  ssSetSFcnParamTunable(S, 35, 0);	// Overcurrent_2
  ssSetSFcnParamTunable(S, 36, 0);	// Step_Mode_2
  ssSetSFcnParamTunable(S, 37, 0);	// Hold_torque_2
  ssSetSFcnParamTunable(S, 38, 0);	// Run_torque_2
  ssSetSFcnParamTunable(S, 39, 0);	// ACC_torque_2
  ssSetSFcnParamTunable(S, 40, 0);	// DEC_torque_2
  
  ssSetSFcnParamTunable(S, 41, 0);	// Current_Voltage_Mode_SEL_2
  ssSetSFcnParamTunable(S, 42, 0);	// Motor_2_SEL
  ssSetSFcnParamTunable(S, 43, 0);	// Motor_3_SEL
  
  ssSetNumPWork(S, 0);

  if (!ssSetNumDWork(S, 0))
    return;

  /*
   * Set the number of input ports.
   */
  if (!ssSetNumInputPorts(S, 0))
    return;

  /*
   * Set the number of output ports.
   */
  if (!ssSetNumOutputPorts(S, 0))
    return;

  /*
   * This S-function can be used in referenced model simulating in normal mode.
   */
  ssSetModelReferenceNormalModeSupport(S, MDL_START_AND_MDL_PROCESS_PARAMS_OK);

  /*
   * Set the number of sample time.
   */
  ssSetNumSampleTimes(S, 1);

  /*
   * All options have the form SS_OPTION_<name> and are documented in
   * matlabroot/simulink/include/simstruc.h. The options should be
   * combined with bitwise OR as in
   *   ssSetOptions(S, (SS_OPTION_name1 | SS_OPTION_name2))
   */
  ssSetOptions(S,
               SS_OPTION_USE_TLC_WITH_ACCELERATOR |
               SS_OPTION_CAN_BE_CALLED_CONDITIONALLY |
               SS_OPTION_EXCEPTION_FREE_CODE |
               SS_OPTION_WORKS_WITH_CODE_REUSE |
               SS_OPTION_SFUNCTION_INLINED_FOR_RTW |
               SS_OPTION_DISALLOW_CONSTANT_SAMPLE_TIME);
}

/* Function: mdlInitializeSampleTimes =====================================
 * Abstract:
 *    This function is used to specify the sample time(s) for your
 *    S-function. You must register the same number of sample times as
 *    specified in ssSetNumSampleTimes.
 */
static void mdlInitializeSampleTimes(SimStruct *S)
{
  ssSetSampleTime(S, 0, INHERITED_SAMPLE_TIME);
  ssSetOffsetTime(S, 0, FIXED_IN_MINOR_STEP_OFFSET);

#if defined(ssSetModelReferenceSampleTimeDefaultInheritance)

  ssSetModelReferenceSampleTimeDefaultInheritance(S);

#endif

}


#define MDL_SET_WORK_WIDTHS
#if defined(MDL_SET_WORK_WIDTHS) && defined(MATLAB_MEX_FILE)

/* Function: mdlSetWorkWidths =============================================
 * Abstract:
 *      The optional method, mdlSetWorkWidths is called after input port
 *      width, output port width, and sample times of the S-function have
 *      been determined to set any state and work vector sizes which are
 *      a function of the input, output, and/or sample times.
 *
 *      Run-time parameters are registered in this method using methods
 *      ssSetNumRunTimeParams, ssSetRunTimeParamInfo, and related methods.
 */
static void mdlSetWorkWidths(SimStruct *S)
{
  /* Set number of run-time parameters */
  if (!ssSetNumRunTimeParams(S, 44))
    return;

  /*
   * Register the run-time parameter 1
   */
  
  ssRegDlgParamAsRunTimeParam(S, 1, 1, "ACC", SS_DOUBLE);
  ssRegDlgParamAsRunTimeParam(S, 2, 2, "DEC", SS_DOUBLE);
  ssRegDlgParamAsRunTimeParam(S, 3, 3, "Max_Speed", ssGetDataTypeId(S, "double"));
  ssRegDlgParamAsRunTimeParam(S, 4, 4, "Min_Speed", SS_DOUBLE);
  ssRegDlgParamAsRunTimeParam(S, 5, 5, "Full_Step_Speed", SS_DOUBLE);
  ssRegDlgParamAsRunTimeParam(S, 6, 6, "Overcurrent", SS_DOUBLE);
  ssRegDlgParamAsRunTimeParam(S, 7, 7, "Step_Mode", ssGetDataTypeId(S, "uint8"));
  ssRegDlgParamAsRunTimeParam(S, 8, 8, "Hold_torque", SS_DOUBLE);
  ssRegDlgParamAsRunTimeParam(S, 9, 9, "Run_torque", SS_DOUBLE);
  ssRegDlgParamAsRunTimeParam(S, 10, 10, "ACC_torque", SS_DOUBLE);
  ssRegDlgParamAsRunTimeParam(S, 11,11 , "DEC_torque", SS_DOUBLE);

  ssRegDlgParamAsRunTimeParam(S, 12, 12, "SPI_Nr", ssGetDataTypeId(S, "uint8"));
  ssRegDlgParamAsRunTimeParam(S, 13, 13, "Current_Voltage_Mode_SEL", ssGetDataTypeId(S, "boolean"));
  ssRegDlgParamAsRunTimeParam(S, 14, 14, "Reset_Port", ssGetDataTypeId(S, "uint8"));
  ssRegDlgParamAsRunTimeParam(S, 15, 15, "Reset_Pin", ssGetDataTypeId(S, "uint8"));
  ssRegDlgParamAsRunTimeParam(S, 16, 16, "SSEL_Port", ssGetDataTypeId(S, "uint8"));
  ssRegDlgParamAsRunTimeParam(S, 17, 17, "SSEL_Pin", ssGetDataTypeId(S, "uint8"));
  ssRegDlgParamAsRunTimeParam(S, 18, 18, "ACC_1", SS_DOUBLE);
  ssRegDlgParamAsRunTimeParam(S, 19, 19, "DEC_1", SS_DOUBLE);
  ssRegDlgParamAsRunTimeParam(S, 20, 20, "Max_Speed_1", ssGetDataTypeId(S, "double"));
  ssRegDlgParamAsRunTimeParam(S, 21, 21, "Min_Speed_1", SS_DOUBLE);
  ssRegDlgParamAsRunTimeParam(S, 22, 22, "Full_Step_Speed_1", SS_DOUBLE);
  ssRegDlgParamAsRunTimeParam(S, 23, 23, "Overcurrent_1", SS_DOUBLE);
  ssRegDlgParamAsRunTimeParam(S, 24, 24, "Step_Mode_1", ssGetDataTypeId(S, "uint8"));
  ssRegDlgParamAsRunTimeParam(S, 25, 25, "Hold_torque_1", SS_DOUBLE);
  ssRegDlgParamAsRunTimeParam(S, 26, 26, "Run_torque_1", SS_DOUBLE);
  ssRegDlgParamAsRunTimeParam(S, 27, 27, "ACC_torque_1", SS_DOUBLE);
  ssRegDlgParamAsRunTimeParam(S, 28,28 , "DEC_torque_1", SS_DOUBLE);


  ssRegDlgParamAsRunTimeParam(S, 29, 29, "Current_Voltage_Mode_SEL_1", ssGetDataTypeId(S, "boolean"));
  ssRegDlgParamAsRunTimeParam(S, 30, 30, "ACC_2", SS_DOUBLE);
  ssRegDlgParamAsRunTimeParam(S, 31, 31, "DEC_2", SS_DOUBLE);
  ssRegDlgParamAsRunTimeParam(S, 32, 32, "Max_Speed_2", ssGetDataTypeId(S, "double"));
  ssRegDlgParamAsRunTimeParam(S, 33, 33, "Min_Speed_2", SS_DOUBLE);
  ssRegDlgParamAsRunTimeParam(S, 34, 34, "Full_Step_Speed_2", SS_DOUBLE);
  ssRegDlgParamAsRunTimeParam(S, 35, 35, "Overcurrent_2", SS_DOUBLE);
  ssRegDlgParamAsRunTimeParam(S, 36, 36, "Step_Mode_2", ssGetDataTypeId(S, "uint8"));
  ssRegDlgParamAsRunTimeParam(S, 37, 37, "Hold_torque_2", SS_DOUBLE);
  ssRegDlgParamAsRunTimeParam(S, 38, 38, "Run_torque_2", SS_DOUBLE);
  ssRegDlgParamAsRunTimeParam(S, 39, 39, "ACC_torque_2", SS_DOUBLE);
  ssRegDlgParamAsRunTimeParam(S, 40, 40 , "DEC_torque_2", SS_DOUBLE);
  ssRegDlgParamAsRunTimeParam(S, 41, 41, "Current_Voltage_Mode_SEL_2", ssGetDataTypeId(S, "boolean"));  

  ssRegDlgParamAsRunTimeParam(S, 42, 42, "Motor_2_SEL", ssGetDataTypeId(S, "boolean"));
  ssRegDlgParamAsRunTimeParam(S, 43, 43, "Motor_3_SEL", ssGetDataTypeId(S, "boolean"));
}

#endif

#define MDL_START
#if defined(MDL_START)

/* Function: mdlStart =====================================================
 * Abstract:
 *    This function is called once at start of model execution. If you
 *    have states that should be initialized once, this is the place
 *    to do it.
 */
static void mdlStart(SimStruct *S)
{
    UNUSED_PARAMETER(S);
}

#endif

/* Function: mdlOutputs ===================================================
 * Abstract:
 *    In this function, you compute the outputs of your S-function
 *    block. Generally outputs are placed in the output vector(s),
 *    ssGetOutputPortSignal.
 */
static void mdlOutputs(SimStruct *S, int_T tid)
{
    UNUSED_PARAMETER(S);
    UNUSED_PARAMETER(tid);
}

/* Function: mdlTerminate =================================================
 * Abstract:
 *    In this function, you should perform any actions that are necessary
 *    at the termination of a simulation.
 */
static void mdlTerminate(SimStruct *S)
{
    UNUSED_PARAMETER(S);
}

/* Function: IsRealMatrix =================================================
 * Abstract:
 *      Verify that the mxArray is a real (double) finite matrix
 */
static bool IsRealMatrix(const mxArray * const m)
{
  if (mxIsNumeric(m) &&
      mxIsDouble(m) &&
      !mxIsLogical(m) &&
      !mxIsComplex(m) &&
      !mxIsSparse(m) &&
      !mxIsEmpty(m) &&
      mxGetNumberOfDimensions(m) == 2) {
    const double * const data = mxGetPr(m);
    const size_t numEl = mxGetNumberOfElements(m);
    size_t i;
    for (i = 0; i < numEl; i++) {
      if (!mxIsFinite(data[i])) {
        return(false);
      }
    }

    return(true);
  } else {
    return(false);
  }
}


/*
 * Required S-function trailer
 */
#ifdef MATLAB_MEX_FILE
# include "simulink.c"
#else
# include "cg_sfun.h"
#endif
