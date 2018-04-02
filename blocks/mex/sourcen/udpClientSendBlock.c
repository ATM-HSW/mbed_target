/*
 * File: udpClientWriteBlock.c
 *
 * Copyright 2017-2018 Dr.O.Hagendorf, HS Wismar
 */

#define S_FUNCTION_NAME  udpClientSendBlock
#define S_FUNCTION_LEVEL 2

#include "simstruc.h"
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
  int_T numInput;
  
  int_T inputPortIdx  = 0;
  int_T outputPortIdx = 0; 
  
  ssSetNumSFcnParams(S, 3);  /* Number of expected parameters */
  if (ssGetNumSFcnParams(S) != ssGetSFcnParamsCount(S)) {
    return;
  }

  ssSetSFcnParamTunable(S, 0, 0);
  ssSetSFcnParamTunable(S, 1, 0);
  ssSetSFcnParamTunable(S, 2, 0);

  /* Register the number and type of states the S-Function uses */
  ssSetNumContStates(    S, 0);   /* number of continuous states           */
  ssSetNumDiscStates(    S, 0);   /* number of discrete states             */

/*
 * Set the number of input ports.
 */
  numInput = (int_T)mxGetScalar(ssGetSFcnParam(S, 2));
  if (!ssSetNumInputPorts(S, numInput?2:1))
    return;

  /*
   * Configure the input ports
   */
  ssSetInputPortDataType(S, 0, SS_UINT8);
  ssSetInputPortWidth(S, 0, DYNAMICALLY_SIZED);
  ssSetInputPortComplexSignal(S, 0, COMPLEX_NO);
  ssSetInputPortDirectFeedThrough(S, 0, 1);
  ssSetInputPortAcceptExprInRTW(S, 0, 0);
  ssSetInputPortOverWritable(S, 0, 0);
  ssSetInputPortOptimOpts(S, 0, SS_REUSABLE_AND_LOCAL);
  ssSetInputPortRequiredContiguous(S, 0, 1);

  if(numInput) {
    ssSetInputPortDataType(S, 1, SS_UINT16);
    ssSetInputPortWidth(S, 1, 1);
    ssSetInputPortComplexSignal(S, 1, COMPLEX_NO);
    ssSetInputPortDirectFeedThrough(S, 1, 1);
    ssSetInputPortAcceptExprInRTW(S, 1, 0);
    ssSetInputPortOverWritable(S, 1, 0);
    ssSetInputPortOptimOpts(S, 1, SS_REUSABLE_AND_LOCAL);
    ssSetInputPortRequiredContiguous(S, 1, 1);
  }

  if (!ssSetNumOutputPorts(S, 0)) return;

  ssSetNumSampleTimes(   S, 1);   /* number of sample times                */

  /*
   * Set size of the work vectors.
   */
  ssSetNumRWork(         S, 0);   /* number of real work vector elements   */
  ssSetNumIWork(         S, 0);   /* number of integer work vector elements*/
  ssSetNumPWork(         S, 0);   /* number of pointer work vector elements*/
  ssSetNumModes(         S, 0);   /* number of mode work vector elements   */
  ssSetNumNonsampledZCs( S, 0);   /* number of nonsampled zero crossings   */

  /* Specify the sim state compliance to be same as a built-in block */
  /* see sfun_simstate.c for example of other possible settings */
  ssSetSimStateCompliance(S, USE_DEFAULT_SIM_STATE);

  /*
   * All options have the form SS_OPTION_<name> and are documented in
   * matlabroot/simulink/include/simstruc.h. The options should be
   * bitwise or'd together as in
   *   ssSetOptions(S, (SS_OPTION_name1 | SS_OPTION_name2))
   */
  ssSetOptions(S,
             SS_OPTION_USE_TLC_WITH_ACCELERATOR |
             SS_OPTION_CAN_BE_CALLED_CONDITIONALLY |
             SS_OPTION_EXCEPTION_FREE_CODE |
             SS_OPTION_WORKS_WITH_CODE_REUSE |
             SS_OPTION_SFUNCTION_INLINED_FOR_RTW |
             SS_OPTION_DISALLOW_CONSTANT_SAMPLE_TIME);   /* general options (SS_OPTION_xx)        */

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
  if (!ssSetNumRunTimeParams(S, 3))
    return;

  /*
   * Register the run-time parameter 1
   */
  ssRegDlgParamAsRunTimeParam(S, 0, 0, "SampleTime",  ssGetDataTypeId(S, "int32"));
  ssRegDlgParamAsRunTimeParam(S, 1, 1, "sock_ID", ssGetDataTypeId(S, "uint8"));
  ssRegDlgParamAsRunTimeParam(S, 2, 2, "enable_number_of_bytes",    ssGetDataTypeId(S, "uint8"));
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
