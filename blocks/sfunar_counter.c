/* Copyright 2010 The MathWorks, Inc. */
/* Copyright 2014 Dr.O.Hagendorf, HS Wismar  */
/* Copyright 2016 M. Marquardt, HS Wismar  */


/*
 * Must specify the S_FUNCTION_NAME as the name of the S-function.
 */
#define S_FUNCTION_NAME                sfunar_counter
#define S_FUNCTION_LEVEL               2

/*
 * Need to include simstruc.h for the definition of the SimStruct and
 * its associated macro definitions.
 */
#include "simstruc.h"

#define EDIT_OK(S, P_IDX) \
 (!((ssGetSimMode(S)==SS_SIMMODE_SIZES_CALL_ONLY) && mxIsEmpty(ssGetSFcnParam(S, P_IDX))))
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
  /*
   * Check the parameter 1 TimerNumber
   */
  if EDIT_OK(S, 0) {
    int_T dimsArray[2] = { 1, 1 };

    /* Check the parameter attributes */
    ssCheckSFcnParamValueAttribs(S, 0, "P1", DYNAMICALLY_TYPED, 2, dimsArray, 0);
  }
  /*
   * Check the parameter 2 InputChannel
   */
  if EDIT_OK(S, 1) {
    int_T dimsArray[2] = { 1, 1 };

    /* Check the parameter attributes */
    ssCheckSFcnParamValueAttribs(S, 1, "P2", DYNAMICALLY_TYPED, 2, dimsArray, 0);
  }
  /*
   * Check the parameter 3 TiPort
   */
  if EDIT_OK(S, 2) {
    int_T dimsArray[2] = { 1, 1 };

    /* Check the parameter attributes */
    ssCheckSFcnParamValueAttribs(S, 2, "P3", DYNAMICALLY_TYPED, 2, dimsArray, 0);
  }
  /*
   * Check the parameter 4 TiPin
   */
  if EDIT_OK(S, 3) {
    int_T dimsArray[2] = { 1, 1 };

    /* Check the parameter attributes */
    ssCheckSFcnParamValueAttribs(S, 3, "P4", DYNAMICALLY_TYPED, 2, dimsArray, 0);
  }
  /*
   * Check the parameter 5 TiPupd
   */
  if EDIT_OK(S, 4) {
    int_T dimsArray[2] = { 1, 1 };

    /* Check the parameter attributes */
    ssCheckSFcnParamValueAttribs(S, 4, "P5", DYNAMICALLY_TYPED, 2, dimsArray, 0);
  }
  /*
   * Check the parameter 6 MaxCnt
   */
  if EDIT_OK(S, 5) {
    int_T dimsArray[2] = { 1, 1 };

    /* Check the parameter attributes */
    ssCheckSFcnParamValueAttribs(S, 5, "P6", DYNAMICALLY_TYPED, 2, dimsArray, 0);
  }
  /*
   * Check the parameter 7 Interrupt
   */
  if EDIT_OK(S, 6) {
    int_T dimsArray[2] = { 1, 1 };

    /* Check the parameter attributes */
    ssCheckSFcnParamValueAttribs(S, 6, "P7", DYNAMICALLY_TYPED, 2, dimsArray, 0);
  }
  /*
   * Check the parameter 8 RstEnable
   */
  if EDIT_OK(S, 7) {
    int_T dimsArray[2] = { 1, 1 };

    /* Check the parameter attributes */
    ssCheckSFcnParamValueAttribs(S, 7, "P8", DYNAMICALLY_TYPED, 2, dimsArray, 0);
  }
}

#endif

/* Function: mdlInitializeSizes ===========================================
 * Abstract:
 *    The sizes information is used by Simulink to determine the S-function
 *    block's characteristics (number of inputs, outputs, states, etc.).
 */
static void mdlInitializeSizes(SimStruct *S)
{
  int i;
  /* Number of expected parameters */
  ssSetNumSFcnParams(S, 8);

  
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
  ssSetSFcnParamTunable(S, 0, 0);	// TimerNumber
  ssSetSFcnParamTunable(S, 1, 0);	// InputChannel
  ssSetSFcnParamTunable(S, 2, 0);	// TiPort
  ssSetSFcnParamTunable(S, 3, 0);	// TiPin
  ssSetSFcnParamTunable(S, 4, 0);	// TiPupd
  ssSetSFcnParamTunable(S, 5, 0);	// MaxCnt
  ssSetSFcnParamTunable(S, 6, 0);	// Interrupt  
  ssSetSFcnParamTunable(S, 7, 0);	// RstEnable

  ssSetNumPWork(S, 0);

  if (!ssSetNumDWork(S, 0))
    return;

  /*
   * Set the number of input ports.
   */
  if(*mxGetPr(ssGetSFcnParam(S, 7)) > 0)
  {
    if (!ssSetNumInputPorts(S, 1))
      return;
	
	ssSetInputPortDataType(S, 0, SS_UINT8);
	ssSetInputPortWidth(S, 0, 1);
	ssSetInputPortComplexSignal(S, 0, COMPLEX_NO);
	ssSetInputPortDirectFeedThrough(S, 0, 1);
	ssSetInputPortAcceptExprInRTW(S, 0, 0);
	ssSetInputPortOverWritable(S, 0, 1);
	ssSetInputPortOptimOpts(S, 0, SS_REUSABLE_AND_LOCAL);
	ssSetInputPortRequiredContiguous(S, 0, 1);
  }
  else
  {	  
    if (!ssSetNumInputPorts(S, 0))
      return;
  }
 
  /*
   * Set the number of output ports.
   */
  
  if(*mxGetPr(ssGetSFcnParam(S, 6)) == 1)
  {
    if (!ssSetNumOutputPorts(S, 1))
      return;   
			  
	ssSetOutputPortDataType(S, 0, SS_UINT32);
	ssSetOutputPortWidth(S, 0, 1);
	ssSetOutputPortComplexSignal(S, 0, COMPLEX_NO);
	ssSetOutputPortOptimOpts(S, 0, SS_REUSABLE_AND_LOCAL);
	ssSetOutputPortOutputExprInRTW(S, 0, 0);
  }
  else if(*mxGetPr(ssGetSFcnParam(S, 6)) == 2)
  {	  
    if (!ssSetNumOutputPorts(S, 2))
      return;   
			  
	ssSetOutputPortDataType(S, 1, SS_UINT32);
	ssSetOutputPortWidth(S, 1, 1);
	ssSetOutputPortComplexSignal(S, 1, COMPLEX_NO);
	ssSetOutputPortOptimOpts(S, 1, SS_REUSABLE_AND_LOCAL);
	ssSetOutputPortOutputExprInRTW(S, 1, 0);
	  
    ssSetOutputPortDataType(S, 0, SS_FCN_CALL);
    ssSetOutputPortWidth(S, 0, 1);
    ssSetOutputPortComplexSignal(S, 0, COMPLEX_NO);
    ssSetOutputPortOptimOpts(S, 0, SS_REUSABLE_AND_LOCAL);
    ssSetOutputPortOutputExprInRTW(S, 0, 1);
  }
  else
  {	  
    if (!ssSetNumOutputPorts(S, 1))
      return; 

    ssSetOutputPortDataType(S, 0, SS_FCN_CALL);
    ssSetOutputPortWidth(S, 0, 1);
    ssSetOutputPortComplexSignal(S, 0, COMPLEX_NO);
    ssSetOutputPortOptimOpts(S, 0, SS_REUSABLE_AND_LOCAL);
    ssSetOutputPortOutputExprInRTW(S, 0, 1);
  }
  
  
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

  if(*mxGetPr(ssGetSFcnParam(S, 6)) > 1)
	ssSetCallSystemOutput(S,0);

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
  if (!ssSetNumRunTimeParams(S, 8))
    return;

  /*
   * Register the run-time parameter 1
   */
  ssRegDlgParamAsRunTimeParam(S, 0, 0, "TimerNumber", ssGetDataTypeId(S, "uint8"));
  ssRegDlgParamAsRunTimeParam(S, 1, 1, "InputChannel", ssGetDataTypeId(S, "uint8"));
  ssRegDlgParamAsRunTimeParam(S, 2, 2, "TiPort", ssGetDataTypeId(S, "uint8"));
  ssRegDlgParamAsRunTimeParam(S, 3, 3, "TiPin", ssGetDataTypeId(S, "uint8"));
  ssRegDlgParamAsRunTimeParam(S, 4, 4, "TiPupd", ssGetDataTypeId(S, "uint8"));
  ssRegDlgParamAsRunTimeParam(S, 5, 5, "MaxCnt", ssGetDataTypeId(S, "uint32"));
  ssRegDlgParamAsRunTimeParam(S, 6, 6, "Interrupt", ssGetDataTypeId(S, "uint8"));  
  ssRegDlgParamAsRunTimeParam(S, 7, 7, "RstEnable", ssGetDataTypeId(S, "uint8"));

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

/*
 * Required S-function trailer
 */
#ifdef MATLAB_MEX_FILE
# include "simulink.c"
#else
# include "cg_sfun.h"
#endif
