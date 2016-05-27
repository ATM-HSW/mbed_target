/* Copyright 2010 The MathWorks, Inc. */
/* Copyright 2016 Dr.O.Hagendorf, HS Wismar  */

/*
 * Must specify the S_FUNCTION_NAME as the name of the S-function.
 */
#define S_FUNCTION_NAME                sfunar_DisplayCR12832
#define S_FUNCTION_LEVEL               2

/*
 * Need to include simstruc.h for the definition of the SimStruct and
 * its associated macro definitions.
 */
#include "simstruc.h"

#define EDIT_OK(S, P_IDX) (!((ssGetSimMode(S)==SS_SIMMODE_SIZES_CALL_ONLY) && mxIsEmpty(ssGetSFcnParam(S, P_IDX))))
#define MDL_CHECK_PARAMETERS
#if defined(MDL_CHECK_PARAMETERS) && defined(MATLAB_MEX_FILE)
#define SAMPLE_TIME                    (ssGetSFcnParam(S, 0))

/*
 * Utility function prototypes.
 */
static bool IsRealMatrix(const mxArray * const m);

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
   * Check the parameter: sample time
   */
  if EDIT_OK(S, 0) {
    const double *sampleTime = NULL;
    const size_t stArraySize = mxGetM(SAMPLE_TIME) * mxGetN(SAMPLE_TIME);

    /* Sample time must be a real scalar value or 2 element array. */
    if (IsRealMatrix(SAMPLE_TIME) && (stArraySize == 1 || stArraySize == 2) ) {
      sampleTime = (real_T *) mxGetPr(SAMPLE_TIME);
    } else {
      ssSetErrorStatus(S, "Invalid sample time. Sample time must be a real scalar value or an array of two real values.");
      return;
    }

    if (sampleTime[0] < 0.0 && sampleTime[0] != -1.0) {
      ssSetErrorStatus(S, "Invalid sample time. Period must be non-negative or -1 (for inherited).");
      return;
    }

    if (stArraySize == 2 && sampleTime[0] > 0.0 && sampleTime[1] >= sampleTime[0]) {
      ssSetErrorStatus(S, "Invalid sample time. Offset must be smaller than period.");
      return;
    }

    if (stArraySize == 2 && sampleTime[0] == -1.0 && sampleTime[1] != 0.0) {
      ssSetErrorStatus(S, "Invalid sample time. When period is -1, offset must be 0.");
      return;
    }

    if (stArraySize == 2 && sampleTime[0] == 0.0 && !(sampleTime[1] == 1.0)) {
      ssSetErrorStatus(S, "Invalid sample time. When period is 0, offset must be 1.");
      return;
    }
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
  int i, j, errorOutputEnable, numberOfVectorElements;
  int_T numberOfInputs1, numberOfInputs2, numberOfInputs3, portcounter;
  int idxTypeInputPorts, numElements;

  /* Number of expected parameters */
  ssSetNumSFcnParams(S, 18);

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

	/*
	SampleTime, 
	TypeOfInputs1, NumberOfInputs1, double(Row1), 
	TypeOfInputs2, NumberOfInputs2, double(Row2), 
	TypeOfInputs3, NumberOfInputs3, double(Row3), 
	uint32(BufferSize), SpiPort, CsPort, CsPin
	*/
  /* Set the parameter's tunable status */
  ssSetSFcnParamTunable(S, 0, 0);
  ssSetSFcnParamTunable(S, 1, 0);
  ssSetSFcnParamTunable(S, 2, 0);
  ssSetSFcnParamTunable(S, 3, 0);
  ssSetSFcnParamTunable(S, 4, 0);
  ssSetSFcnParamTunable(S, 5, 0);
  ssSetSFcnParamTunable(S, 6, 0);
  ssSetSFcnParamTunable(S, 7, 0);
  ssSetSFcnParamTunable(S, 8, 0);
  ssSetSFcnParamTunable(S, 9, 0);
  ssSetSFcnParamTunable(S,10, 0);
  ssSetSFcnParamTunable(S,11, 0);
  ssSetSFcnParamTunable(S,12, 0);
  ssSetSFcnParamTunable(S,13, 0);
  ssSetSFcnParamTunable(S,14, 0);
  ssSetSFcnParamTunable(S,15, 0);
  ssSetSFcnParamTunable(S,16, 0);
  ssSetSFcnParamTunable(S,17, 0);

  ssSetNumPWork(S, 0);

  if (!ssSetNumDWork(S, 0))
    return;

  /*
   * Set the number of input ports.
   */
  
  numberOfInputs1  = (int_T)mxGetNumberOfElements(ssGetSFcnParam(S, 1));
  numberOfInputs2  = (int_T)mxGetNumberOfElements(ssGetSFcnParam(S, 4));
  numberOfInputs3  = (int_T)mxGetNumberOfElements(ssGetSFcnParam(S, 7));
  if (!ssSetNumInputPorts(S, numberOfInputs1+numberOfInputs2+numberOfInputs3))
    return;
 	//printf("%d %d %d\r\n", numberOfInputs1, numberOfInputs2, numberOfInputs3);

  /*
   * Configure the input ports
   */
  portcounter = 0;
  for(idxTypeInputPorts = 1; idxTypeInputPorts < 8; idxTypeInputPorts+=3) {
  	j = idxTypeInputPorts==1?numberOfInputs1:(idxTypeInputPorts==2?numberOfInputs2:numberOfInputs3);
	  for (i = 0; i < j; i++) {
	    //printf("%d %d %d %d: %s\r\n", portcounter, idxTypeInputPorts, i, j, mxArrayToString(mxGetCell(ssGetSFcnParam(S, idxTypeInputPorts), i)));
	    if (strstr(mxArrayToString(mxGetCell(ssGetSFcnParam(S, idxTypeInputPorts), i)), "uint8"))
	      ssSetInputPortDataType(S, portcounter, SS_UINT8);
	    else if (strstr(mxArrayToString(mxGetCell(ssGetSFcnParam(S, idxTypeInputPorts), i)), "uint16"))
	      ssSetInputPortDataType(S, portcounter, SS_UINT16);
	    else if (strstr(mxArrayToString(mxGetCell(ssGetSFcnParam(S, idxTypeInputPorts), i)), "uint32"))
	      ssSetInputPortDataType(S, portcounter, SS_UINT32);
	    else if (strstr(mxArrayToString(mxGetCell(ssGetSFcnParam(S, idxTypeInputPorts), i)), "int8"))
	      ssSetInputPortDataType(S, portcounter, SS_INT8);
	    else if (strstr(mxArrayToString(mxGetCell(ssGetSFcnParam(S, idxTypeInputPorts), i)), "int16"))
	      ssSetInputPortDataType(S, portcounter, SS_INT16);
	    else if (strstr(mxArrayToString(mxGetCell(ssGetSFcnParam(S, idxTypeInputPorts), i)), "int32"))
	      ssSetInputPortDataType(S, portcounter, SS_INT32);
	    else if (strstr(mxArrayToString(mxGetCell(ssGetSFcnParam(S, idxTypeInputPorts), i)), "single"))
	      ssSetInputPortDataType(S, portcounter, SS_SINGLE);
	    else if (strstr(mxArrayToString(mxGetCell(ssGetSFcnParam(S, idxTypeInputPorts), i)), "double"))
	      ssSetInputPortDataType(S, portcounter, SS_DOUBLE);
	    else if (strstr(mxArrayToString(mxGetCell(ssGetSFcnParam(S, idxTypeInputPorts), i)), "bool"))
	      ssSetInputPortDataType(S, portcounter, SS_BOOLEAN);
	    else
	      mexWarnMsgTxt("One or more Ports Datatypes not set");

	    ssSetInputPortWidth(S, portcounter, 1);
	    ssSetInputPortComplexSignal(S, portcounter, COMPLEX_NO);
	    ssSetInputPortDirectFeedThrough(S, portcounter, 1);
	    ssSetInputPortAcceptExprInRTW(S, portcounter, 1);
	    ssSetInputPortOverWritable(S, portcounter, 1);
	    ssSetInputPortOptimOpts(S, portcounter, SS_REUSABLE_AND_LOCAL);
	    ssSetInputPortRequiredContiguous(S, portcounter, 1);
	    portcounter++;
	  }
	}

  /*
   * Inits the output ports.
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
  const double * const sampleTime = mxGetPr(SAMPLE_TIME);
  const size_t stArraySize = mxGetM(SAMPLE_TIME) * mxGetN(SAMPLE_TIME);
  ssSetSampleTime(S, 0, sampleTime[0]);
  if (stArraySize == 1) {
    ssSetOffsetTime(S, 0, (sampleTime[0] == CONTINUOUS_SAMPLE_TIME ? FIXED_IN_MINOR_STEP_OFFSET : 0.0));
  } else {
    ssSetOffsetTime(S, 0, sampleTime[1]);
  }

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
  if (!ssSetNumRunTimeParams(S, 18))
    return;

	/*
	SampleTime, 
	TypeOfInputs1, NumberOfInputs1, double(Row1), 
	TypeOfInputs2, NumberOfInputs2, double(Row2), 
	TypeOfInputs3, NumberOfInputs3, double(Row3), 
	uint32(BufferSize), SpiPort, CsPort, CsPin
	*/
  /*
  * Register the run-time parameter
  */
  ssRegDlgParamAsRunTimeParam(S, 0, 0, "SampleTime",   ssGetDataTypeId(S, "int32"));
//  ssRegDlgParamAsRunTimeParam(S, 1, 1, "typeinpports1", ssGetDataTypeId(S, "uint8"));
  ssRegDlgParamAsRunTimeParam(S, 2, 2, "NumberOfInputs1", ssGetDataTypeId(S, "uint8"));
  ssRegDlgParamAsRunTimeParam(S, 3, 3, "Row1", ssGetDataTypeId(S, "uint8"));
//  ssRegDlgParamAsRunTimeParam(S, 4, 4, "typeinpports2", ssGetDataTypeId(S, "uint8"));
  ssRegDlgParamAsRunTimeParam(S, 5, 5, "NumberOfInputs2", ssGetDataTypeId(S, "uint8"));
  ssRegDlgParamAsRunTimeParam(S, 6, 6, "Row2", ssGetDataTypeId(S, "uint8"));
//  ssRegDlgParamAsRunTimeParam(S, 7, 7, "typeinpports3", ssGetDataTypeId(S, "uint8"));
  ssRegDlgParamAsRunTimeParam(S, 8, 8, "NumberOfInputs3", ssGetDataTypeId(S, "uint8"));
  ssRegDlgParamAsRunTimeParam(S, 9, 9, "Row3", ssGetDataTypeId(S, "uint8"));
  ssRegDlgParamAsRunTimeParam(S,10,10, "BufferSize",   ssGetDataTypeId(S, "uint32"));
  ssRegDlgParamAsRunTimeParam(S,11,11, "SpiPort",      ssGetDataTypeId(S, "uint8"));
  ssRegDlgParamAsRunTimeParam(S,12,12, "CsPort",       ssGetDataTypeId(S, "uint8"));
  ssRegDlgParamAsRunTimeParam(S,13,13, "CsPin",        ssGetDataTypeId(S, "uint8"));
  ssRegDlgParamAsRunTimeParam(S,14,14, "ResPort",       ssGetDataTypeId(S, "uint8"));
  ssRegDlgParamAsRunTimeParam(S,15,15, "ResPin",        ssGetDataTypeId(S, "uint8"));
  ssRegDlgParamAsRunTimeParam(S,16,16, "A0Port",       ssGetDataTypeId(S, "uint8"));
  ssRegDlgParamAsRunTimeParam(S,17,17, "A0Pin",        ssGetDataTypeId(S, "uint8"));
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
    if (mxIsNumeric(m)  &&  
        mxIsDouble(m)   && 
        !mxIsLogical(m) &&
        !mxIsComplex(m) &&  
        !mxIsSparse(m)  && 
        !mxIsEmpty(m)   &&
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

