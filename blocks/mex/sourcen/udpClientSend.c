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

#define S_FUNCTION_NAME  udpClientSend
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



/*=====================================*
 * Configuration and execution methods *
 *=====================================*/
static void mdlInitializeSizes(SimStruct *S)
{
    int i;
    int_T nInputPorts  = 0;  /* number of input ports  */

    ssSetNumSFcnParams(S, 3);  /* Number of expected parameters */
    if (ssGetNumSFcnParams(S) != ssGetSFcnParamsCount(S)) return;

    /* 
     * Configure tunability of parameters.  By default, all parameters are
     * tunable (changeable) during simulation.  If there are parameters that 
     * cannot change during simulation, such as any parameters that would change 
     * the number of ports on the block, the sample time of the block, or the 
     * data type of a signal, mark these as non-tunable using a call like this:
     * 
     *    ssSetSFcnParamTunable(S, 0, 0);
     *
     * which sets parameter 0 to be non-tunable (0).
     *
     */
    ssSetSFcnParamTunable(S, 0, 0);
    ssSetSFcnParamTunable(S, 1, 0);
    ssSetSFcnParamTunable(S, 2, 0);

    /* Register the number and type of states the S-Function uses */

    ssSetNumContStates(    S, 0);   /* number of continuous states           */
    ssSetNumDiscStates(    S, 0);   /* number of discrete states             */
    
    // Now we need two now how many Inputports we must set
    nInputPorts  = (int_T)mxGetNumberOfElements(ssGetSFcnParam(S,2));

    /*
     * Configure the input ports. First set the number of input ports. 
     */
    if (!ssSetNumInputPorts(S, nInputPorts)) return;

  for ( i = 0; i < nInputPorts; i++)
  {

    ssSetInputPortWidth(S, i, 1);
    
    if (strstr(mxArrayToString(mxGetCell(ssGetSFcnParam(S, 2), i)), "uint8"))
      ssSetInputPortDataType(S, i, SS_UINT8);
    else if (strstr(mxArrayToString(mxGetCell(ssGetSFcnParam(S, 2), i)), "int8"))
      ssSetInputPortDataType(S, i, SS_INT8);
    else if (strstr(mxArrayToString(mxGetCell(ssGetSFcnParam(S, 2), i)), "uint16"))
      ssSetInputPortDataType(S, i, SS_UINT16);
    else if (strstr(mxArrayToString(mxGetCell(ssGetSFcnParam(S, 2), i)), "int16"))
      ssSetInputPortDataType(S, i, SS_INT16);
    else if (strstr(mxArrayToString(mxGetCell(ssGetSFcnParam(S, 2), i)), "uint32"))
      ssSetInputPortDataType(S, i, SS_UINT32);
    else if (strstr(mxArrayToString(mxGetCell(ssGetSFcnParam(S, 2), i)), "int32"))
      ssSetInputPortDataType(S, i, SS_INT32);
    else if (strstr(mxArrayToString(mxGetCell(ssGetSFcnParam(S, 2), i)), "single"))
      ssSetInputPortDataType(S, i, SS_SINGLE);
    else if (strstr(mxArrayToString(mxGetCell(ssGetSFcnParam(S, 2), i)), "double"))
      ssSetInputPortDataType(S, i, SS_DOUBLE);
    else
      mexWarnMsgTxt("udpClientSend: One or more Ports Datatypes not set");
    ssSetInputPortWidth(S, i, 1);
    ssSetInputPortComplexSignal(S, i, COMPLEX_NO);
    ssSetInputPortDirectFeedThrough(S, i, 1);
    ssSetInputPortAcceptExprInRTW(S, i, 0);
    ssSetInputPortOverWritable(S, i, 1);
    ssSetInputPortOptimOpts(S, i, SS_REUSABLE_AND_LOCAL);
    ssSetInputPortRequiredContiguous(S, i, 1);
    //if (!ssSetInputPortDimensionInfo(S, i, DYNAMICALLY_SIZED)) return;
    //mxFree(str);
  }

    /*
     * Configure the output ports. First set the number of output ports.
     */
    if (!ssSetNumOutputPorts(S, 0)) return;

    /*
     * Set the number of sample times. This must be a positive, nonzero
     * integer indicating the number of sample times or it can be
     * PORT_BASED_SAMPLE_TIMES. For multi-rate S-functions, the
     * suggested approach to setting sample times is via the port
     * based sample times method. When you create a multirate
     * S-function, care needs to be taking to verify that when
     * slower tasks are preempted that your S-function correctly
     * manages data as to avoid race conditions. When port based
     * sample times are specified, the block cannot inherit a constant
     * sample time at any port.
     */
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


#define MDL_SET_WORK_WIDTHS   /* Change to #undef to remove function */
#if defined(MDL_SET_WORK_WIDTHS) && defined(MATLAB_MEX_FILE)
  /* Function: mdlSetWorkWidths ===============================================
   * Abstract:
   *      The optional method, mdlSetWorkWidths is called after input port
   *      width, output port width, and sample times of the S-function have
   *      been determined to set any state and work vector sizes which are
   *      a function of the input, output, and/or sample times. This method
   *      is used to specify the nonzero work vector widths via the macros
   *      ssNumContStates, ssSetNumDiscStates, ssSetNumRWork, ssSetNumIWork,
   *      ssSetNumPWork, ssSetNumModes, and ssSetNumNonsampledZCs.
   *
   *      Run-time parameters are registered in this method using methods 
   *      ssSetNumRunTimeParams, ssSetRunTimeParamInfo, and related methods.
   *
   *      If you are using mdlSetWorkWidths, then any work vectors you are
   *      using in your S-function should be set to DYNAMICALLY_SIZED in
   *      mdlInitializeSizes, even if the exact value is known at that point.
   *      The actual size to be used by the S-function should then be specified
   *      in mdlSetWorkWidths.
   */
  static void mdlSetWorkWidths(SimStruct *S)
  {
  /* Set number of run-time parameters */
  if (!ssSetNumRunTimeParams(S, 1))
    return;

  /*
   * Register the run-time parameter 1
   */
  ssRegDlgParamAsRunTimeParam(S, 1, 0, "sock_ID", ssGetDataTypeId(S, "uint8"));
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
} /* end mdlOutputs */


/* Function: mdlTerminate =====================================================
 * Abstract:
 *    In this function, you should perform any actions that are necessary
 *    at the termination of a simulation.  For example, if memory was allocated
 *    in mdlStart, this is the place to free it.
 *
 *    Suppose your S-function allocates a few few chunks of memory in mdlStart
 *    and saves them in PWork. The following code fragment would free this
 *    memory.
 *        {
 *            int i;
 *            for (i = 0; i<ssGetNumPWork(S); i++) {
 *                if (ssGetPWorkValue(S,i) != NULL) {
 *                    free(ssGetPWorkValue(S,i));
 *                }
 *            }
 *        }
 */
static void mdlTerminate(SimStruct *S)
{
}


#define MDL_RTW  /* Change to #undef to remove function */
#if defined(MDL_RTW) && defined(MATLAB_MEX_FILE)
  /* Function: mdlRTW =========================================================
   * Abstract:
   *
   *    This function is called when the Real-Time Workshop is generating
   *    the model.rtw file. In this method, you can call the following
   *    functions which add fields to the model.rtw file.
   *
   *    1) The following creates Parameter records for your S-functions.
   *       nParams is the number of tunable S-function parameters.
   *
   *       if ( !ssWriteRTWParameters(S, nParams,
   *
   *                                  SSWRITE_VALUE_[type],paramName,stringInfo,
   *                                  [type specific arguments below]
   *
   *                                 ) ) {
   *           return; (error reporting will be handled by SL)
   *       }
   *
   *       Where SSWRITE_VALUE_[type] can be one of the following groupings
   *       (and you must have "nParams" such groupings):
   *
   *         SSWRITE_VALUE_VECT,
   *           const char_T *paramName,
   *           const char_T *stringInfo,
   *           const real_T *valueVect,
   *           int_T        vectLen
   *
   *         SSWRITE_VALUE_2DMAT,
   *           const char_T *paramName,
   *           const char_T *stringInfo,
   *           const real_T *valueMat,
   *           int_T        nRows,
   *           int_T        nCols
   *
   *         SSWRITE_VALUE_DTYPE_VECT,
   *           const char_T   *paramName,
   *           const char_T   *stringInfo,
   *           const void     *valueVect,
   *           int_T          vectLen,
   *           int_T          dtInfo
   *
   *         SSWRITE_VALUE_DTYPE_2DMAT,
   *           const char_T   *paramName,
   *           const char_T   *stringInfo,
   *           const void     *valueMat,
   *           int_T          nRows,
   *           int_T          nCols,
   *           int_T          dtInfo
   *
   *         SSWRITE_VALUE_DTYPE_ML_VECT,
   *           const char_T   *paramName,
   *           const char_T   *stringInfo,
   *           const void     *rValueVect,
   *           const void     *iValueVect,
   *           int_T          vectLen,
   *           int_T          dtInfo
   *
   *         SSWRITE_VALUE_DTYPE_ML_2DMAT,
   *           const char_T   *paramName,
   *           const char_T   *stringInfo,
   *           const void     *rValueMat,
   *           const void     *iValueMat,
   *           int_T          nRows,
   *           int_T          nCols,
   *           int_T          dtInfo
   *
   *       Notes:
   *       1. nParams is an integer and stringInfo is a string describing
   *          generalinformation about the parameter such as how it was derived.
   *       2. The last argument to this function, dtInfo, is obtained from the
   *          DTINFO macro (defined in simstruc.h) as:
   *                 dtInfo = DTINFO(dataTypeId, isComplexSignal);
   *          where dataTypeId is the data type id and isComplexSignal is a
   *          boolean value specifying whether the parameter is complex.
   *
   *       See simulink/include/simulink.c for the definition (implementation)
   *       of this function and simulink/src/sfun_multiport.c for an example
   *       of using this function.
   *
   *    2) The following creates SFcnParameterSetting record for S-functions
   *       (these can be derived from the non-tunable S-function parameters).
   *
   *       if ( !ssWriteRTWParamSettings(S, nParamSettings,
   *
   *                                     SSWRITE_VALUE_[whatever], settingName,
   *                                     [type specific arguments below]
   *
   *                                    ) ) {
   *           return; (error reporting will be handled by SL)
   *       }
   *
   *       Where SSWRITE_VALUE_[type] can be one of the following groupings
   *       (and you must have "nParamSettings" such groupings):
   *       Also, the examples in the right hand column below show how the
   *       ParamSetting appears in the .rtw file
   *
   *         SSWRITE_VALUE_STR,              - Used to write (un)quoted strings
   *           const char_T *settingName,      example:
   *           const char_T *value,              Country      USA
   *
   *         SSWRITE_VALUE_QSTR,             - Used to write quoted strings
   *           const char_T *settingName,      example:
   *           const char_T *value,              Country      "U.S.A"
   *
   *         SSWRITE_VALUE_VECT_STR,         - Used to write vector of strings
   *           const char_T *settingName,      example:
   *           const char_T *value,              Countries    ["USA", "Mexico"]
   *           int_T        nItemsInVect
   *
   *         SSWRITE_VALUE_NUM,              - Used to write numbers
   *           const char_T *settingName,      example:
   *           const real_T value                 NumCountries  2
   *
   *
   *         SSWRITE_VALUE_VECT,             - Used to write numeric vectors
   *           const char_T *settingName,      example:
   *           const real_T *settingValue,       PopInMil        [300, 100]
   *           int_T        vectLen
   *
   *         SSWRITE_VALUE_2DMAT,            - Used to write 2D matrices
   *           const char_T *settingName,      example:
   *           const real_T *settingValue,       PopInMilBySex  Matrix(2,2)
   *           int_T        nRows,                   [[170, 130],[60, 40]]
   *           int_T        nCols
   *
   *         SSWRITE_VALUE_DTYPE_NUM,        - Used to write numeric vectors
   *           const char_T   *settingName,    example: int8 Num 3+4i
   *           const void     *settingValue,   written as: [3+4i]
   *           int_T          dtInfo
   *
   *
   *         SSWRITE_VALUE_DTYPE_VECT,       - Used to write data typed vectors
   *           const char_T   *settingName,    example: int8 CArray [1+2i 3+4i]
   *           const void     *settingValue,   written as:
   *           int_T          vectLen             CArray  [1+2i, 3+4i]
   *           int_T          dtInfo
   *
   *
   *         SSWRITE_VALUE_DTYPE_2DMAT,      - Used to write data typed 2D
   *           const char_T   *settingName     matrices
   *           const void     *settingValue,   example:
   *           int_T          nRow ,            int8 CMatrix  [1+2i 3+4i; 5 6]
   *           int_T          nCols,            written as:
   *           int_T          dtInfo               CMatrix         Matrix(2,2)
   *                                                [[1+2i, 3+4i]; [5+0i, 6+0i]]
   *
   *
   *         SSWRITE_VALUE_DTYPE_ML_VECT,    - Used to write complex matlab data
   *           const char_T   *settingName,    typed vectors example:
   *           const void     *settingRValue,  example: int8 CArray [1+2i 3+4i]
   *           const void     *settingIValue,      settingRValue: [1 3]
   *           int_T          vectLen              settingIValue: [2 4]
   *           int_T          dtInfo
   *                                             written as:
   *                                                CArray    [1+2i, 3+4i]
   *
   *         SSWRITE_VALUE_DTYPE_ML_2DMAT,   - Used to write matlab complex
   *           const char_T   *settingName,    data typed 2D matrices
   *           const void     *settingRValue,  example
   *           const void     *settingIValue,      int8 CMatrix [1+2i 3+4i; 5 6]
   *           int_T          nRows                settingRValue: [1 5 3 6]
   *           int_T          nCols,               settingIValue: [2 0 4 0]
   *           int_T          dtInfo
   *                                              written as:
   *                                              CMatrix         Matrix(2,2)
   *                                                [[1+2i, 3+4i]; [5+0i, 6+0i]]
   *
   *       Note, The examples above show how the ParamSetting is written out
   *       to the .rtw file
   *
   *       See simulink/include/simulink.c for the definition (implementation)
   *       of this function and simulink/src/sfun_multiport.c for an example
   *       of using this function.
   *
   *    3) The following creates the work vector records for S-functions
   *
   *       if (!ssWriteRTWWorkVect(S, vectName, nNames,
   *
   *                            name, size,   (must have nNames of these pairs)
   *                                 :
   *                           ) ) {
   *           return;  (error reporting will be handled by SL)
   *       }
   *
   *       Notes:
   *         a) vectName must be either "RWork", "IWork" or "PWork"
   *         b) nNames is an int_T (integer), name is a const char_T* (const
   *            char pointer) and size is int_T, and there must be nNames number
   *            of [name, size] pairs passed to the function.
   *         b) intSize1+intSize2+ ... +intSizeN = ssGetNum<vectName>(S)
   *            Recall that you would have to set ssSetNum<vectName>(S)
   *            in one of the initialization functions (mdlInitializeSizes
   *            or mdlSetWorkVectorWidths).
   *
   *       See simulink/include/simulink.c for the definition (implementation)
   *       of this function, and ... no example yet :(
   *
   *    4) Finally the following functions/macros give you the ability to write
   *       arbitrary strings and [name, value] pairs directly into the .rtw
   *       file.
   *
   *       if (!ssWriteRTWStr(S, const_char_*_string)) {
   *          return;
   *       }
   *
   *       if (!ssWriteRTWStrParam(S, const_char_*_name, const_char_*_value)) {
   *          return;
   *       }
   *
   *       if (!ssWriteRTWScalarParam(S, const_char_*_name, 
   *                                  const_void_*_value,
   *                                  DTypeId_dtypeId)) {
   *          return;
   *       }
   *
   *       if (!ssWriteRTWStrVectParam(S, const_char_*_name,
   *                                   const_char_*_value,
   *                                   int_num_items)) {
   *          return;
   *       }
   *
   *       if (!ssWriteRTWVectParam(S, const_char_*_name, const_void_*_value,
   *                                int_data_type_of_value, int_vect_len)){
   *          return;
   *       }
   *
   *       if (!ssWriteRTW2dMatParam(S, const_char_*_name, const_void_*_value,
   *                        int_data_type_of_value, int_nrows, int_ncols)){
   *          return;
   *       }
   *
   *       The 'data_type_of_value' input argument for the above two macros is
   *       obtained using
   *          DTINFO(dTypeId, isComplex),
   *       where
   *          dTypeId: can be any one of the enum values in BuitlInDTypeID
   *                   (SS_DOUBLE, SS_SINGLE, SS_INT8, SS_UINT8, SS_INT16,
   *                   SS_UINT16, SS_INT32, SS_UINT32, SS_BOOLEAN defined
   *                   in simstuc_types.h)
   *          isComplex: is either 0 or 1, as explained in Note-2 for
   *                    ssWriteRTWParameters.
   *
   *       For example DTINFO(SS_INT32,0) is a non-complex 32-bit signed
   *       integer.
   *
   *       If isComplex==1, then it is assumed that 'const_void_*_value' array
   *       has the real and imaginary parts arranged in an interleaved manner
   *       (i.e., Simulink Format).
   *
   *       If you prefer to pass the real and imaginary parts as two seperate
   *       arrays, you should use the follwing macros:
   *
   *       if (!ssWriteRTWMxVectParam(S, const_char_*_name,
   *                                  const_void_*_rvalue, const_void_*_ivalue,
   *                                  int_data_type_of_value, int_vect_len)){
   *          return;
   *       }
   *
   *       if (!ssWriteRTWMx2dMatParam(S, const_char_*_name,
   *                                   const_void_*_rvalue, const_void_*_ivalue,
   *                                   int_data_type_of_value,
   *                                   int_nrows, int_ncols)){
   *          return;
   *       }
   *
   *       See simulink/include/simulink.c and simstruc.h for the definition 
   *       (implementation) of these functions and simulink/src/ml2rtw.c for 
   *       examples of using these functions.
   *
   */
  static void mdlRTW(SimStruct *S)
  {
  int i;
  real_T  nBufferBytes = 0;

  //Now Calculate the Bytes of the Send Buffer
  for (i = 0; i < mxGetNumberOfElements(ssGetSFcnParam(S, 2)); i++)
  {
    if (strstr(mxArrayToString(mxGetCell(ssGetSFcnParam(S, 2), i)), "int8"))
      nBufferBytes++;
    else if (strstr(mxArrayToString(mxGetCell(ssGetSFcnParam(S, 2), i)), "uint8"))
      nBufferBytes++;
    else if (strstr(mxArrayToString(mxGetCell(ssGetSFcnParam(S, 2), i)), "int16"))
      nBufferBytes += 2;
    else if (strstr(mxArrayToString(mxGetCell(ssGetSFcnParam(S, 2), i)), "uint16"))
      nBufferBytes += 2;
    else if (strstr(mxArrayToString(mxGetCell(ssGetSFcnParam(S, 2), i)), "int32"))
      nBufferBytes += 4;
    else if (strstr(mxArrayToString(mxGetCell(ssGetSFcnParam(S, 2), i)), "uint32"))
      nBufferBytes += 4;
    else if (strstr(mxArrayToString(mxGetCell(ssGetSFcnParam(S, 2), i)), "uint32"))
      nBufferBytes += 4;
    else if (strstr(mxArrayToString(mxGetCell(ssGetSFcnParam(S, 2), i)), "single"))
      nBufferBytes += 4;
    else if (strstr(mxArrayToString(mxGetCell(ssGetSFcnParam(S, 2), i)), "double"))
      nBufferBytes += 8;
    else
      mexWarnMsgTxt("udpClientSend: One or more Ports Datatypes not set");
  }

  //now we wanted to write some Parameters to the rtw-file and
  //use them for sending data to the right destination
  if (!ssWriteRTWParamSettings(S, 1,
    SSWRITE_VALUE_NUM, "NBUFFERBYTES", (nBufferBytes+1)))
    return;
  }
#endif /* MDL_RTW */

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
