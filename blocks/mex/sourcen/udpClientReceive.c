/*
 * File: udpClientReceive.c
 *
 * Copyright 2017-2018 Dr.O.Hagendorf, HS Wismar
 */

#define S_FUNCTION_NAME  udpClientReceive
#define S_FUNCTION_LEVEL 2

#include "simstruc.h"
#include "matrix.h"
#define EDIT_OK(S, P_IDX) \
 (!((ssGetSimMode(S)==SS_SIMMODE_SIZES_CALL_ONLY) && mxIsEmpty(ssGetSFcnParam(S, P_IDX))))
#define SAMPLE_TIME  (ssGetSFcnParam(S, 0))

/*
 * Utility function prototypes.
 */
static bool IsRealMatrix(const mxArray * const m);

/*====================================================================*
 * Parameter handling methods. These methods are not supported by RTW *
 *====================================================================*/
#if defined(MATLAB_MEX_FILE)
static void mdlCheckParameters(SimStruct *S)
{
  /*
   * Check the parameter 1 (sample time)
   */
  if EDIT_OK(S, 0) {
    const double *sampleTime = NULL;
    const size_t stArraySize = mxGetM(SAMPLE_TIME) * mxGetN(SAMPLE_TIME);

    /* Sample time must be a real scalar value or 2 element array. */
    if (IsRealMatrix(SAMPLE_TIME) &&
        (stArraySize == 1 || stArraySize == 2) ) {
      sampleTime = (real_T *) mxGetPr(SAMPLE_TIME);
    } else {
      ssSetErrorStatus(S,
                       "Invalid sample time. Sample time must be a real scalar value or an array of two real values.");
      return;
    }

    if (sampleTime[0] < 0.0 && sampleTime[0] != -1.0) {
      ssSetErrorStatus(S,
                       "Invalid sample time. Period must be non-negative or -1 (for inherited).");
      return;
    }

    if (stArraySize == 2 && sampleTime[0] > 0.0 &&
        sampleTime[1] >= sampleTime[0]) {
      ssSetErrorStatus(S,
                       "Invalid sample time. Offset must be smaller than period.");
      return;
    }

    if (stArraySize == 2 && sampleTime[0] == -1.0 && sampleTime[1] != 0.0) {
      ssSetErrorStatus(S,
                       "Invalid sample time. When period is -1, offset must be 0.");
      return;
    }

    if (stArraySize == 2 && sampleTime[0] == 0.0 &&
        !(sampleTime[1] == 1.0)) {
      ssSetErrorStatus(S,
                       "Invalid sample time. When period is 0, offset must be 1.");
      return;
    }
  }
}
#endif /* MDL_CHECK_PARAMETERS */

/*=====================================*
 * Configuration and execution methods *
 *=====================================*/
static void mdlInitializeSizes(SimStruct *S)
{
  int errorOutputEnable;
  int i;
  int_T nOutputPorts = 0;  /* number of output ports */

  ssSetNumSFcnParams(S, 6);  /* Number of expected parameters */
  if (ssGetNumSFcnParams(S) != ssGetSFcnParamsCount(S)) return;

  ssSetSFcnParamTunable(S, 0, 0);
  ssSetSFcnParamTunable(S, 1, 0);
  ssSetSFcnParamTunable(S, 2, 0);
  ssSetSFcnParamTunable(S, 3, 0);
  ssSetSFcnParamTunable(S, 4, 0);
  ssSetSFcnParamTunable(S, 5, 0);

  /* Register the number and type of states the S-Function uses */

  ssSetNumContStates(    S, 0);   /* number of continuous states           */
  ssSetNumDiscStates(    S, 0);   /* number of discrete states             */

  /*
   * Configure the input ports. First set the number of input ports.
   */
  if (!ssSetNumInputPorts(S, 0)) return;

  // Now we need two know how many Outputports we must set
  nOutputPorts  = mxGetNumberOfElements(ssGetSFcnParam(S,2));
  errorOutputEnable = mxGetScalar(ssGetSFcnParam(S,5));

  /*
   * Configure the output ports. First set the number of output ports.
   */
  if (errorOutputEnable) {
    if (!ssSetNumOutputPorts(S, nOutputPorts+1)) return;
  } else {
    if (!ssSetNumOutputPorts(S, nOutputPorts)) return;
  }

  /*
   * Set output port dimensions for each output port index starting at 0.
   * See comments for setting input port dimensions.
   */
  for(i = 0; i < nOutputPorts; i++) {
    ssSetOutputPortWidth(S, i, 1);

    if (strstr(mxArrayToString(mxGetCell(ssGetSFcnParam(S, 2), i)), "uint8"))
      ssSetOutputPortDataType(S, i, SS_UINT8);
    else if (strstr(mxArrayToString(mxGetCell(ssGetSFcnParam(S, 2), i)), "int8"))
      ssSetOutputPortDataType(S, i, SS_INT8);
    else if (strstr(mxArrayToString(mxGetCell(ssGetSFcnParam(S, 2), i)), "uint16"))
      ssSetOutputPortDataType(S, i, SS_UINT16);
    else if (strstr(mxArrayToString(mxGetCell(ssGetSFcnParam(S, 2), i)), "int16"))
      ssSetOutputPortDataType(S, i, SS_INT16);
    else if (strstr(mxArrayToString(mxGetCell(ssGetSFcnParam(S, 2), i)), "uint32"))
      ssSetOutputPortDataType(S, i, SS_UINT32);
    else if (strstr(mxArrayToString(mxGetCell(ssGetSFcnParam(S, 2), i)), "int32"))
      ssSetOutputPortDataType(S, i, SS_INT32);
    else if (strstr(mxArrayToString(mxGetCell(ssGetSFcnParam(S, 2), i)), "single"))
      ssSetOutputPortDataType(S, i, SS_SINGLE);
    else if (strstr(mxArrayToString(mxGetCell(ssGetSFcnParam(S, 2), i)), "double"))
      ssSetOutputPortDataType(S, i, SS_DOUBLE);
    else
      mexWarnMsgTxt("udpClientReceive: One or more Ports Datatypes not set");
    ssSetOutputPortWidth(S, i, 1);
    ssSetOutputPortComplexSignal(S, i, COMPLEX_NO);
    ssSetOutputPortOptimOpts(S, i, SS_REUSABLE_AND_LOCAL);
    ssSetOutputPortOutputExprInRTW(S, i, 1);
  }
  if(errorOutputEnable) {
    ssSetOutputPortDataType(S, i, SS_BOOLEAN);
    ssSetOutputPortWidth(S, i, 1);
    ssSetOutputPortComplexSignal(S, i, COMPLEX_NO);
    ssSetOutputPortOptimOpts(S, i, SS_REUSABLE_AND_LOCAL);
    ssSetOutputPortOutputExprInRTW(S, i, 1);
  }

  ssSetNumSampleTimes(   S, 1);   /* number of sample times                */

  ssSetOptions(S,
             SS_OPTION_USE_TLC_WITH_ACCELERATOR |
             SS_OPTION_CAN_BE_CALLED_CONDITIONALLY |
             SS_OPTION_EXCEPTION_FREE_CODE |
             SS_OPTION_WORKS_WITH_CODE_REUSE |
             SS_OPTION_SFUNCTION_INLINED_FOR_RTW |
             SS_OPTION_DISALLOW_CONSTANT_SAMPLE_TIME);    /* general options (SS_OPTION_xx)        */

} /* end mdlInitializeSizes */


static void mdlInitializeSampleTimes(SimStruct *S)
{
  double * const sampleTime = mxGetPr(SAMPLE_TIME);
  const  size_t stArraySize = mxGetM(SAMPLE_TIME) * mxGetN(SAMPLE_TIME);
  ssSetSampleTime(S, 0, sampleTime[0]);
  if (stArraySize == 1) {
    ssSetOffsetTime(S, 0, (sampleTime[0] == CONTINUOUS_SAMPLE_TIME?
      FIXED_IN_MINOR_STEP_OFFSET: 0.0));
  } else {
    ssSetOffsetTime(S, 0, sampleTime[1]);
  }

#if defined(ssSetModelReferenceSampleTimeDefaultInheritance)

  ssSetModelReferenceSampleTimeDefaultInheritance(S);

#endif

} /* end mdlInitializeSampleTimes */

#define MDL_SET_WORK_WIDTHS
#if defined(MDL_SET_WORK_WIDTHS) && defined(MATLAB_MEX_FILE)
static void mdlSetWorkWidths(SimStruct *S) {
  /* Set number of run-time parameters */
  if (!ssSetNumRunTimeParams(S, 4))
    return;

  /*
   * Register the run-time parameters
   */
  ssRegDlgParamAsRunTimeParam(S, 1, 0, "sock_ID", ssGetDataTypeId(S, "uint8"));
  ssRegDlgParamAsRunTimeParam(S, 3, 1, "BufferSize", ssGetDataTypeId(S, "uint16"));
  ssRegDlgParamAsRunTimeParam(S, 4, 2, "Blocking", ssGetDataTypeId(S, "uint16"));
  ssRegDlgParamAsRunTimeParam(S, 5, 3, "ErrorOutputport", ssGetDataTypeId(S, "boolean"));
  }
#endif /* MDL_SET_WORK_WIDTHS */

static void mdlOutputs(SimStruct *S, int_T tid)
{
} /* end mdlOutputs */

static void mdlTerminate(SimStruct *S)
{
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

/*=============================*
 * Required S-function trailer *
 *=============================*/

#ifdef  MATLAB_MEX_FILE    /* Is this file being compiled as a MEX-file? */
#include "simulink.c"      /* MEX-file interface mechanism */
#else
#include "cg_sfun.h"       /* Code generation registration function */
#endif
