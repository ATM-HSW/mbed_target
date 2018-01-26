/*
 * File: sfuntmpl_doc.c
 * Abstract:
 *       A 'C' template for a level 2 S-function. 
 *
 * Copyright 1990-2013 The MathWorks, Inc.
 */


/*
 * You must specify the S_FUNCTION_NAME as the name of your S-function.
 */

#define S_FUNCTION_NAME  coapRequestGet
#define S_FUNCTION_LEVEL 2

#include "simstruc.h"
#define EDIT_OK(S, P_IDX) \
 (!((ssGetSimMode(S)==SS_SIMMODE_SIZES_CALL_ONLY) && mxIsEmpty(ssGetSFcnParam(S, P_IDX))))
#define SAMPLE_TIME  (ssGetSFcnParam(S, 0))

#define HOST_IP  (mxArrayToString(ssGetSFcnParam(S,1)))
//#define ID       (mxArrayToString(ssGetSFcnParam(S,2)))
#define PATH     (mxArrayToString(ssGetSFcnParam(S,3)))
//#define METHOD   (mxArrayToString(ssGetSFcnParam(S,4)))


/*
 * Utility function prototypes.
 */
 
// For Sample Time 
static bool IsRealMatrix(const mxArray * const m);


/*====================================================================*
 * Parameter handling methods. These methods are not supported by RTW *
 *====================================================================*/

#define MDL_CHECK_PARAMETERS   /* Change to #undef to remove function */
#if defined(MDL_CHECK_PARAMETERS) && defined(MATLAB_MEX_FILE)
static void mdlCheckParameters(SimStruct *S) {
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


static void mdlInitializeSizes(SimStruct *S) {
 /* Number of expected parameters */
  ssSetNumSFcnParams(S, 5);

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

  ssSetSFcnParamTunable(S, 0, 0);  //sampletime
  ssSetSFcnParamTunable(S, 1, 0);  //IP
  ssSetSFcnParamTunable(S, 2, 0);  //ID
  ssSetSFcnParamTunable(S, 3, 0);  //PATH
  ssSetSFcnParamTunable(S, 4, 0);  //METHOD


  if (!ssSetNumInputPorts(S, 0)) return;

  /*
   * Configure the input port 1
   */

  /*
   * Configure the output ports. First set the number of output ports.
   */
  if (!ssSetNumOutputPorts(S,0)) return;

  ssSetNumSampleTimes(   S, 1);   /* number of sample times */



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


#define MDL_SET_WORK_WIDTHS   /* Change to #undef to remove function */
#if defined(MDL_SET_WORK_WIDTHS) && defined(MATLAB_MEX_FILE)

  static void mdlSetWorkWidths(SimStruct *S)
  {
  /* Set number of run-time parameters */
  if (!ssSetNumRunTimeParams(S, 2))
    return;

  /* Register run-time parameters  */
  ssRegDlgParamAsRunTimeParam(S, 2, 0, "MsgID", ssGetDataTypeId(S, "uint8"));
  ssRegDlgParamAsRunTimeParam(S, 4, 1, "Method", ssGetDataTypeId(S, "uint8"));

  }
#endif /* MDL_SET_WORK_WIDTHS */

/* Function: mdlOutputs =======================================================
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

static void mdlTerminate(SimStruct *S)
{
    UNUSED_PARAMETER(S);
}


#define MDL_RTW  /* Change to #undef to remove function */
#if defined(MDL_RTW) && defined(MATLAB_MEX_FILE)

static void mdlRTW(SimStruct *S)
{

//  if(!ssWriteRTWParamSettings(S,4,
//                                SSWRITE_VALUE_QSTR, "hostIP", IP,
//                                SSWRITE_VALUE_QSTR, "msg_id", ID,
//                                SSWRITE_VALUE_QSTR, "path", PATH))
//  {
//    return;
//  }
  if(!ssWriteRTWParamSettings(S, 2,
                                SSWRITE_VALUE_QSTR, "IP", HOST_IP,
                                SSWRITE_VALUE_QSTR, "Path", PATH))
    return;
}
#endif /* MDL_RTW */

/* Function: IsRealMatrix =================================================
 * Abstract:
 *      Verify that the mxArray is a real (double) finite matrix
 */
 // for Sample Time 
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
