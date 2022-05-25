/*
 * File : sfun_csparam.cpp
 * Abstract:
 *   Example of context-sensitive parameters.
 *
 *   This S-Function accepts a single real input signal (which can be scalar
 *   or vector or matrix).  It multiplies the inputs by the value of the
 *   gain parameter (which must be a scalar real number).
 *
 *   This S-function does not support signals with boolean or fixed-point
 *   data types.
 *
 *   If the data type of the gain parameter is:
 *   double     ==> the block treats the parameter as "context-sensitive"
 *                  (it uses the signal data type for the parameter).
 *   non-double ==> the block does not change the parameter data type
 *                  but it may add a data type cast in the generated code.
 *
 * NOTE:
 *   If the parameter is treated as "context-sensitive" and the signal uses
 *   a user-defined / alias data type, the parameter is registered with the
 *   user-defined data type.
 *
 */

/*   Copyright 1990-2014 The MathWorks, Inc. */

/*
 * You must specify the S_FUNCTION_NAME as the name of your S-function.
 */

#define S_FUNCTION_NAME  caoxing_MPC_demo_SF
#define S_FUNCTION_LEVEL 2

/*
 * Need to include simstruc.h for the definition of the SimStruct and
 * its associated macro definitions.
 */
#include <stdio.h>
#include <string.h>
#include "osqp.h"
#include <iostream>
#include <Eigen/SVD>
#include <Eigen/Core>
#include "mpc_osqp.h"

#ifdef MATLAB_MEX_FILE
#include "tmwtypes.h"
#else
#include "rtwtypes.h"
#endif
#include "simstruc.h"
using Matrix = Eigen::MatrixXd;

# define M_PI		3.14159265358979323846	/* pi */
const int8_t basic_state_size = 6;
const int8_t controls = 2; 
/* S-Function parameter indices */
typedef enum {
    GAIN_IDX = 0,
    NPARAMS
} ParamIdx;

#define GAIN_PARAM(S) ssGetSFcnParam(S,GAIN_IDX)
const int8_t InNum=15;
/* ERROR MESSAGES */
const char *invalidSignalDataTypeMsg = 
"Boolean and fixed-point data types not supported.";

#if !defined(MATLAB_MEX_FILE)
/*
 * This file cannot be used directly with the Real-Time Workshop. However,
 * this S-function does work with the Real-Time Workshop via
 * the Target Language Compiler technology. See 
 * matlabroot/toolbox/simulink/blocks/tlc_c/sfun_multiport.tlc   
 */
# error This_file_can_be_used_only_during_simulation_inside_Simulink
#endif


/*====================*
 * S-function methods *
 *====================*/

#define MDL_CHECK_PARAMETERS
#if defined(MDL_CHECK_PARAMETERS) && defined(MATLAB_MEX_FILE)
  /* Function: mdlCheckParameters =============================================
   * Abstract:
   *    Validate our parameters to verify they are okay.
   */
  static void mdlCheckParameters(SimStruct *S)
  {
      /* Check gain parameter */
      {
          if ((mxIsComplex(GAIN_PARAM(S))) ||
              (mxGetNumberOfElements(GAIN_PARAM(S)) != 1)) {
              ssSetErrorStatus(S,"Gain parameter must be a scalar real number");
          }
      }
      return;
  }
#endif /* MDL_CHECK_PARAMETERS */


/* Function: mdlInitializeSizes ===============================================
 * Abstract:
 *    Call mdlCheckParameters to verify that the parameters are okay,
 *    then setup sizes of the various vectors.
 */
static void mdlInitializeSizes(SimStruct *S)
{

    /* Set up parameters */
    ssSetNumSFcnParams(S, NPARAMS);  /* Number of expected parameters */
#if defined(MATLAB_MEX_FILE)
    if (ssGetNumSFcnParams(S) == ssGetSFcnParamsCount(S)) {
        mdlCheckParameters(S);
        if (ssGetErrorStatus(S) != NULL) {
            return;
        }
    } else {
        return; /* Parameter mismatch will be reported by Simulink */
    }

#endif

    ssSetSFcnParamTunable(S, GAIN_IDX, true);

    /* Set up states */
    ssSetNumContStates(S, 0);
    ssSetNumDiscStates(S, 0);

    /* Set up inputs */
    if (!ssSetNumInputPorts(S, InNum)) return;
    for(int8_t i=0; i<InNum; ++i){
        ssSetInputPortWidth(S, i, DYNAMICALLY_SIZED);
        ssSetInputPortDataType(S, i, DYNAMICALLY_TYPED);
        ssSetInputPortOptimOpts(S, i, SS_REUSABLE_AND_GLOBAL);
        ssSetInputPortOverWritable(S, i, true);
        ssSetInputPortAcceptExprInRTW(S, i, true);
        ssSetInputPortDirectFeedThrough(S, i, true);
    }
    ssSetInputPortWidth(S, 0, 1);//basic_state_size
    ssSetInputPortWidth(S, 1, 1);//controls
    ssSetInputPortWidth(S, 2, basic_state_size*basic_state_size);//matrix_a, basic_state_size, basic_state_size
    ssSetInputPortWidth(S, 3, basic_state_size*controls);//matrix_b, basic_state_size, controls
    ssSetInputPortWidth(S, 4, basic_state_size*basic_state_size);//matrix_q, basic_state_size, basic_state_size
    ssSetInputPortWidth(S, 5, controls*controls);//matrix_r, controls, controls
    ssSetInputPortWidth(S, 6, controls*1);//lower_bound, controls, 1
    ssSetInputPortWidth(S, 7, controls*1);//upper_bound, controls, 1
    ssSetInputPortWidth(S, 8, basic_state_size*1);//lower_state_bound, basic_state_size, 1
    ssSetInputPortWidth(S, 9, basic_state_size*1);//upper_state_bound, basic_state_size,1
    ssSetInputPortWidth(S, 10, basic_state_size*1);//reference_state, basic_state_size,1
    ssSetInputPortWidth(S, 11, 1);//mpc_max_iteration, 1
    ssSetInputPortWidth(S, 12, 1);//horizon, 1
    ssSetInputPortWidth(S, 13, 1);//mpc_eps, 1
    ssSetInputPortWidth(S, 14, basic_state_size*1);//reserved, 1

    

    /* Set up outputs */
    if (!ssSetNumOutputPorts(S, 1)) return;
    // ssSetOutputPortWidth(S, 0, DYNAMICALLY_SIZED);
    ssSetOutputPortWidth(S, 0, 1);
    ssSetOutputPortDataType(S, 0, DYNAMICALLY_TYPED);
    ssSetOutputPortOptimOpts(S, 0, SS_REUSABLE_AND_GLOBAL);
    ssSetOutputPortOutputExprInRTW(S, 0, true);

    /* Set up sample times */
    ssSetNumSampleTimes(S, 1);

    /* Set up work vectors */
    ssSetNumRWork(S, 0);
    ssSetNumIWork(S, 0);
    ssSetNumPWork(S, 0);
    ssSetNumModes(S, 0);
    ssSetNumNonsampledZCs(S, 0);

    /* specify the operating point compliance to be same as a built-in block */
    ssSetOperatingPointCompliance(S, USE_DEFAULT_OPERATING_POINT);

    /* Set S-Function options - NOTE ESPECIALLY:
     * - SS_OPTION_SUPPORTS_ALIAS_DATA_TYPES
     *   (supports user-defined / alias data types)
     * - SS_OPTION_SFUNCTION_INLINED_FOR_RTW
     *   (use TLC code to inline S-Function in generated code)
     */
    ssSetOptions(S,
                 SS_OPTION_SUPPORTS_ALIAS_DATA_TYPES |
                 SS_OPTION_SFUNCTION_INLINED_FOR_RTW |
                 SS_OPTION_WORKS_WITH_CODE_REUSE |
                 SS_OPTION_EXCEPTION_FREE_CODE |
                 SS_OPTION_ALLOW_INPUT_SCALAR_EXPANSION |
                 SS_OPTION_USE_TLC_WITH_ACCELERATOR | 
                 SS_OPTION_CALL_TERMINATE_ON_EXIT);
}


/* Function: mdlInitializeSampleTimes =========================================
 * Abstract:
 *    Specify that we inherit our sample time from the driving block.
 */
static void mdlInitializeSampleTimes(SimStruct *S)
{
    ssSetSampleTime(S, 0, INHERITED_SAMPLE_TIME);
    ssSetOffsetTime(S, 0, 0.0);
    ssSetModelReferenceSampleTimeDefaultInheritance(S);
}


#define MDL_SET_WORK_WIDTHS   /* Change to #undef to remove function */
#if defined(MDL_SET_WORK_WIDTHS) && defined(MATLAB_MEX_FILE)
/* Function: mdlSetWorkWidths ===============================================
 * Abstract:
 *      Set up run-time parameters.
 */
static void mdlSetWorkWidths(SimStruct *S)
{
    /* Set number of run-time parameters */
    if (!ssSetNumRunTimeParams(S, 1)) return;

    /* Register run-time parameters */
    DTypeId actualType = ssGetOutputPortDataType(S,0);
    ssRegDlgParamAsRunTimeParam(S, GAIN_IDX, 0, "Gain", actualType);

    /* NOTE:
     * - We have turned on SS_OPTION_SUPPORTS_ALIAS_DATA_TYPES so
     *   ssGetOutputPortDataType will return the actual data type
     *   (which may be a user-defined / alias data type). */
}
#endif /* MDL_SET_WORK_WIDTHS */


#define MDL_PROCESS_PARAMETERS   /* Change to #undef to remove function */
#if defined(MDL_PROCESS_PARAMETERS) && defined(MATLAB_MEX_FILE)
/* Function: mdlProcessParameters ===========================================
 * Abstract:
 *      Update run-time parameters.
 */
static void mdlProcessParameters(SimStruct *S)
{
    /* Update Run-Time parameter values */
    ssUpdateDlgParamAsRunTimeParam(S, GAIN_IDX);
    
    /* NOTE: We could also have used
     * - ssUpdateAllTunableParamsAsRunTimeParams(S);
     */
}
#endif /* MDL_PROCESS_PARAMETERS */


/* Function: fcnOutputs ======================================================
 * Abstract:
 *   Generic function to implement mdlOutputs.  We have defined this as a
 *   template function so that it can be called for all data types.
 *
 *      y[i] = "gain" * u[i];
 */
template <typename NumericType>
void fcnOutputs(SimStruct *S, int_T tid)
{
    int_T idx;
    int_T width = ssGetInputPortWidth(S, 0);
    InputRealPtrsType uPtrs = ssGetInputPortRealSignalPtrs(S,0);
    NumericType *y1  =  (NumericType *) (ssGetOutputPortRealSignal(S,0));
    NumericType gain = *(NumericType *) (ssGetRunTimeParamInfo(S,0)->data);

    UNUSED_PARAMETER(tid);

    for (idx=0; idx<width; idx++) {
        const NumericType *u1 = (const NumericType * const) uPtrs[idx];
        y1[idx] = gain * (*u1);
    }
}


/* Function: mdlOutputs =======================================================
 * Abstract:
 *   Wrapper function to set up the call to the correct version of
 *   fcnOutputs (based on the data type of the input/output signal).
 */
static void mdlOutputs(SimStruct *S, int_T tid)
{
    /* We need to get the base (aliased-through) data type */
    DTypeId baseType = ssGetDataTypeIdAliasedThruTo(S, ssGetOutputPortDataType(S,0));

    /* For each possible data type, call the appropriate outputs function.
     *
     * OPTIMIZATION: Switch out the S-Function's mdlOutputs function
     * (so that it gets called instead of this function in future).
     */
    switch (baseType) {
      case SS_DOUBLE:
        fcnOutputs<real_T>(S,tid);
        ssSetmdlOutputs(S, &(fcnOutputs<real_T>));
        break;
      case SS_SINGLE:
        fcnOutputs<real32_T>(S,tid);
        ssSetmdlOutputs(S, &(fcnOutputs<real32_T>));
        break;
      case SS_INT32:
        fcnOutputs<int32_T>(S,tid);
        ssSetmdlOutputs(S, &(fcnOutputs<int32_T>));
        break;
      case SS_INT16:
        fcnOutputs<int16_T>(S,tid);
        ssSetmdlOutputs(S, &(fcnOutputs<int16_T>));
        break;
      case SS_INT8:
        fcnOutputs<int8_T>(S,tid);
        ssSetmdlOutputs(S, &(fcnOutputs<int8_T>));
        break;
      case SS_UINT32:
        fcnOutputs<uint32_T>(S,tid);
        ssSetmdlOutputs(S, &(fcnOutputs<uint32_T>));
        break;
      case SS_UINT16:
        fcnOutputs<uint16_T>(S,tid);
        ssSetmdlOutputs(S, &(fcnOutputs<uint16_T>));
        break;
      case SS_UINT8:
        fcnOutputs<uint8_T>(S,tid);
        ssSetmdlOutputs(S, &(fcnOutputs<uint8_T>));
        break;
      default:
        ssSetErrorStatus(S, "Unsupported data type");
    }

    // self development
    // Eigen Test
    {
        Eigen::MatrixXd eig_A(2, 2);
        // // eig_A(0,0) = *A[0];
        eig_A(0,0) = 0;
        eig_A(0,1) = 1;
        eig_A(1,0) = 2;
        eig_A(1,1) = 3;
    }
    int_T idx,idy;
    int_T width; 
    // matrix_a input port 2
    Eigen::MatrixXd matrix_a = Matrix::Zero(basic_state_size, basic_state_size);
    width = ssGetInputPortWidth(S, 2);
    InputRealPtrsType uPtrs_matrix_a = ssGetInputPortRealSignalPtrs(S,2);
    for(idx=0; idx<basic_state_size; ++idx){
        for(idy=0; idy<basic_state_size; ++idy){
            matrix_a(idx, idy) = *uPtrs_matrix_a[idx*basic_state_size + idy];
        }
    }
    //matrix_b, basic_state_size, controls input port 3
    Eigen::MatrixXd matrix_b = Matrix::Zero(basic_state_size, controls);
    width = ssGetInputPortWidth(S, 3);
    InputRealPtrsType uPtrs_matrix_b = ssGetInputPortRealSignalPtrs(S,3);
    for(idx=0; idx<basic_state_size; ++idx){
        for(idy=0; idy<controls; ++idy){
            matrix_b(idx, idy) = *uPtrs_matrix_b[idx*controls + idy];
        }
    }
    // //matrix_q, basic_state_size, basic_state_size input port 4
    Eigen::MatrixXd matrix_q = Matrix::Zero(basic_state_size, basic_state_size);
    width = ssGetInputPortWidth(S, 4);
    InputRealPtrsType uPtrs_matrix_q = ssGetInputPortRealSignalPtrs(S,4);
    for(idx=0; idx<basic_state_size; ++idx){
        for(idy=0; idy<basic_state_size; ++idy){
            matrix_q(idx, idy) = *uPtrs_matrix_q[idx*basic_state_size + idy];
        }
    }
    //matrix_r, controls, controls input port 5
    Eigen::MatrixXd matrix_r = Matrix::Zero(controls, controls);
    width = ssGetInputPortWidth(S, 5);
    InputRealPtrsType uPtrs_matrix_r = ssGetInputPortRealSignalPtrs(S,5);
    for(idx=0; idx<controls; ++idx){
        for(idy=0; idy<controls; ++idy){
            matrix_r(idx, idy) = *uPtrs_matrix_r[idx*controls + idy];
        }
    }
    // //lower_bound, controls, 1 input port 6
    Eigen::MatrixXd lower_bound = Matrix::Zero(controls, 1);
    width = ssGetInputPortWidth(S, 6);
    InputRealPtrsType uPtrs_matrix_lb = ssGetInputPortRealSignalPtrs(S,6);
    for(idx=0; idx<controls; ++idx){
        for(idy=0; idy<1; ++idy){
            lower_bound(idx, idy) = *uPtrs_matrix_lb[idx + idy];
        }
    }
    // //upper_bound, controls, 1 input port 7
    Eigen::MatrixXd upper_bound = Matrix::Zero(controls, 1);
    width = ssGetInputPortWidth(S, 7);
    InputRealPtrsType uPtrs_matrix_ub = ssGetInputPortRealSignalPtrs(S,7);
    for(idx=0; idx<controls; ++idx){
        for(idy=0; idy<1; ++idy){
            upper_bound(idx, idy) = *uPtrs_matrix_ub[idx + idy];
        }
    }
    // //lower_state_bound, basic_state_size, 1 input port 8
    Eigen::MatrixXd lower_state_bound = Matrix::Zero(basic_state_size, 1);
    width = ssGetInputPortWidth(S, 8);
    InputRealPtrsType uPtrs_matrix_lsb = ssGetInputPortRealSignalPtrs(S,8);
    for(idx=0; idx<basic_state_size; ++idx){
        for(idy=0; idy<1; ++idy){
            lower_state_bound(idx, idy) = *uPtrs_matrix_lsb[idx + idy];
        }
    }
    // // //upper_state_bound, basic_state_size,1 input port 9
    Eigen::MatrixXd upper_state_bound = Matrix::Zero(basic_state_size, 1);
    width = ssGetInputPortWidth(S, 9);
    InputRealPtrsType uPtrs_matrix_usb = ssGetInputPortRealSignalPtrs(S,9);
    for(idx=0; idx<basic_state_size; ++idx){
        for(idy=0; idy<1; ++idy){
            upper_state_bound(idx, idy) = *uPtrs_matrix_usb[idx + idy];
        }
    }
    // //reference_state, basic_state_size,1 input port 10
    Eigen::MatrixXd reference_state = Matrix::Zero(basic_state_size, 1);
    width = ssGetInputPortWidth(S, 10);
    InputRealPtrsType uPtrs_matrix_ref = ssGetInputPortRealSignalPtrs(S,10);
    for(idx=0; idx<basic_state_size; ++idx){
        for(idy=0; idy<1; ++idy){
            reference_state(idx, idy) = *uPtrs_matrix_ref[idx + idy];
        }
    }
    //mpc_max_iteration, 1 input port 11
    InputRealPtrsType uPtrs_matrix_maxIter = ssGetInputPortRealSignalPtrs(S,11);
    int mpc_max_iteration = *uPtrs_matrix_maxIter[0];
    //horizon, 1 input port 12
    InputRealPtrsType uPtrs_matrix_horizon = ssGetInputPortRealSignalPtrs(S,12);
    int horizon = *uPtrs_matrix_horizon[0];
    //mpc_eps, 1 input port 13
    InputRealPtrsType uPtrs_matrix_mpcEps = ssGetInputPortRealSignalPtrs(S,13);
    double mpc_eps = *uPtrs_matrix_mpcEps[0];
    // //reference_state, basic_state_size,1 input port 14
    Eigen::MatrixXd matrix_state = Matrix::Zero(basic_state_size, 1);
    width = ssGetInputPortWidth(S, 14);
    InputRealPtrsType uPtrs_matrix_state = ssGetInputPortRealSignalPtrs(S,14);
    for(idx=0; idx<basic_state_size; ++idx){
        for(idy=0; idy<1; ++idy){
            matrix_state(idx, idy) = *uPtrs_matrix_state[idx + idy];
        }
    }
    MpcOsqp mpc_osqp_solver(matrix_a, 
                            matrix_b, 
                            matrix_q, 
                            matrix_r, 
                            matrix_state, 
                            lower_bound,
                            upper_bound, 
                            lower_state_bound,
                            upper_state_bound, 
                            reference_state, 
                            mpc_max_iteration, 
                            horizon,
                            mpc_eps);
        // auto start_time_osqp = std::chrono::system_clock::now();
    std::vector<double> control_cmd(controls, 0);
    mpc_osqp_solver.Solve(&control_cmd);
    
    // {
    //     const int states = 4;
    //     const int controls = 2;
    //     const int horizon = 3;
    //     const int max_iter = 100;
    //     const double eps = 0.001;
    //     const double max = std::numeric_limits<double>::max();

    //     Eigen::MatrixXd A(states, states);
    //     A << 5., 6., 7., 8., 7., 8., 7., 8., 9., 10., 7., 8., 11., 4., 7., 8.;

    //     Eigen::MatrixXd B(states, controls);
    //     B << 2., 3, 2., 7, 2, 9, 3, 8;

    //     Eigen::MatrixXd Q(states, states);
    //     Q << 10., 0, 0, 0, 0, 10., 0, 0, 0, 0, 10.0, 0, 0, 0, 0, 10.0;

    //     Eigen::MatrixXd R(controls, controls);
    //     R << 0.1, 0, 0, 0.1;

    //     Eigen::MatrixXd lower_control_bound(controls, 1);
    //     lower_control_bound << 9.6 - 10.5916, 9.6 - 10.5916;

    //     Eigen::MatrixXd upper_control_bound(controls, 1);
    //     upper_control_bound << 13 - 10.5916, 13 - 10.5916;

    //     Eigen::MatrixXd lower_state_bound(states, 1);
    //     lower_state_bound << -M_PI / 6, -M_PI / 6, -1 * max, -1 * max;

    //     Eigen::MatrixXd upper_state_bound(states, 1);
    //     upper_state_bound << M_PI / 6, M_PI / 6, max, max;

    //     Eigen::MatrixXd initial_state(states, 1);
    //     initial_state << 0, 0, 0, 0;
    //     std::vector<double> control_cmd(controls, 0);

    //     Eigen::MatrixXd reference_state(states, 1);
    //     reference_state << 0, 0, 1, 0;
    //     std::vector<Eigen::MatrixXd> reference(horizon, reference_state);

    //     // OSQP
    //     MpcOsqp mpc_osqp_solver(A, B, Q, R, initial_state, lower_control_bound,
    //                             upper_control_bound, lower_state_bound,
    //                             upper_state_bound, reference_state, max_iter, horizon,
    //                             eps);
    //     // auto start_time_osqp = std::chrono::system_clock::now();
    //     mpc_osqp_solver.Solve(&control_cmd);
    // }
}



/* Function: mdlTerminate =====================================================
 * Abstract:
 *      Free the user data.
 */
static void mdlTerminate(SimStruct *S)
{
    /* No action required */

    UNUSED_PARAMETER(S);
}


/* Function: isAcceptableDataType
 *    determine if the data type ID corresponds to an unsigned integer
 */
static boolean_T isAcceptableDataType(SimStruct *S, DTypeId dType)
{
    /* We need to get the base (aliased-through) data type */
    DTypeId baseType = ssGetDataTypeIdAliasedThruTo(S, dType);

    boolean_T isAcceptable = false;

    switch (baseType) {
      case SS_DOUBLE:
      case SS_SINGLE:
      case SS_INT32:
      case SS_INT16:
      case SS_INT8:
      case SS_UINT32:
      case SS_UINT16:
      case SS_UINT8:
        isAcceptable = true;
        break;
    }

    return isAcceptable;
}


#ifdef MATLAB_MEX_FILE

#define MDL_SET_INPUT_PORT_DATA_TYPE
/* Function: mdlSetInputPortDataType ==========================================
 *    Validate the input/output data types.
 */
static void mdlSetInputPortDataType(SimStruct *S, 
                                    int       port, 
                                    DTypeId   dataType)
{
    if (isAcceptableDataType(S, dataType)) {
        for(int8_t i=0; i<InNum; ++i){
            ssSetInputPortDataType (S, i, dataType);
        }
        ssSetOutputPortDataType(S, 0, dataType);
    } else {
        /* Reject proposed data type */
        ssSetErrorStatus(S, invalidSignalDataTypeMsg);
        goto EXIT_POINT;
    }

EXIT_POINT:
    return;
} /* mdlSetInputPortDataType */


#define MDL_SET_OUTPUT_PORT_DATA_TYPE
/* Function: mdlSetOutputPortDataType =========================================
 *    Validate the input/output data types.
 */
static void mdlSetOutputPortDataType(SimStruct *S, 
                                     int       port, 
                                     DTypeId   dataType)
{
    if (isAcceptableDataType(S, dataType)) {
        ssSetInputPortDataType (S, 0, dataType);
        ssSetOutputPortDataType(S, 0, dataType);
    } else {
        /* Reject proposed data type */
        ssSetErrorStatus(S, invalidSignalDataTypeMsg);
        goto EXIT_POINT;
    }

EXIT_POINT:
    return;

} /* mdlSetOutputPortDataType */

#define MDL_SET_DEFAULT_PORT_DATA_TYPES
/* Function: mdlSetDefaultPortDataTypes ========================================
 *    Set default input/output data types.
 */
static void mdlSetDefaultPortDataTypes(SimStruct *S)
{
    /* Set input port data type to double */
    ssSetInputPortDataType (S, 0, SS_DOUBLE);
    ssSetOutputPortDataType(S, 0, SS_DOUBLE);

} /* mdlSetDefaultPortDataTypes */

#endif /* MATLAB_MEX_FILE */


/* Function: mdlRTW
 * Abstract:
 *   This function is not needed, because we set the option:
 *
 *          SS_OPTION_SFUNCTION_INLINED_FOR_RTW
 */

/*=============================*
 * Required S-function trailer *
 *=============================*/

#ifdef  MATLAB_MEX_FILE    /* Is this file being compiled as a MEX-file? */
#include "simulink.c"      /* MEX-file interface mechanism */
#else
#include "cg_sfun.h"       /* Code generation registration function */
#endif
