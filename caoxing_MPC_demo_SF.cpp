#define S_FUNCTION_NAME  caoxing_MPC_demo_SF
#define S_FUNCTION_LEVEL 2

#include <stdio.h>
#include <string.h>
#include "osqp/osqp.h"
#include <iostream>
#include <Eigen/SVD>
#include <Eigen/Core>
#include "mpc_osqp.h"
#include "simstruc.h"

using Matrix = Eigen::MatrixXd;
enum {PARAM = 0, NUM_PARAMS=2};//parameter number
const int_T Num_Ins = 13;//input number
#define PARAM_ARG ssGetSFcnParam(S, PARAM)

#define EDIT_OK(S, ARG) \
(!((ssGetSimMode(S) == SS_SIMMODE_SIZES_CALL_ONLY) && mxIsEmpty(ARG)))


#ifdef MATLAB_MEX_FILE
#define MDL_CHECK_PARAMETERS
/* Function: mdlCheckParameters =============================================
 * Abstract:
 *    Verify parameter settings.
 */
static void mdlCheckParameters(SimStruct *S)
{
    if(EDIT_OK(S, PARAM_ARG)){
        /* Check that parameter value is not empty*/
        if( mxIsEmpty(PARAM_ARG) ) {
            ssSetErrorStatus(S, "Invalid parameter specified. The parameter "
            "must be non-empty");
            return;
        }
    }
} /* end mdlCheckParameters */
#endif

/* Function: mdlInitializeSizes ===============================================
 * Abstract:
 *   Initialize the sizes array
 */
static void mdlInitializeSizes(SimStruct *S)
{
    ssSetNumSFcnParams(S, NUM_PARAMS);
    
    #if defined(MATLAB_MEX_FILE)
    if (ssGetNumSFcnParams(S) != ssGetSFcnParamsCount(S)) return;
    mdlCheckParameters(S);
    if (ssGetErrorStatus(S) != NULL) return;
    #endif
    
    {
    int iParam = 0;
    int nParam = ssGetNumSFcnParams(S);
    
    for ( iParam = 0; iParam < nParam; iParam++ )
       {
        ssSetSFcnParamTunable( S, iParam, SS_PRM_TUNABLE );
       }
    }
    
    /* Allow signal dimensions greater than 2 */
    ssAllowSignalsWithMoreThan2D(S);
    
    /* Set number of input and output ports */
    if (!ssSetNumInputPorts(S,Num_Ins)) return;
    if (!ssSetNumOutputPorts(S,1)) return;
    
    /* Set dimensions of input and output ports */
    {
        for(int i=0; i<Num_Ins; ++i){
            if(!ssSetInputPortDimensionInfo( S, i, DYNAMIC_DIMENSION)) return;
            ssSetInputPortDirectFeedThrough(S, i, 1);
        }
        // output dimension
        DECL_AND_INIT_DIMSINFO(do0);
        int_T dims_o0[2];
        do0.numDims = 2;
        dims_o0[0] = 1;
        dims_o0[1] = 1;
        do0.dims = dims_o0;
        do0.width = 1;
        if(!ssSetOutputPortDimensionInfo(S, 0, &do0)) return;
    }
    
    ssSetNumSampleTimes(S, 1);

    /* specify the sim state compliance to be same as a built-in block */
    ssSetOperatingPointCompliance(S, USE_DEFAULT_OPERATING_POINT);

    ssSetOptions(S,
    SS_OPTION_WORKS_WITH_CODE_REUSE |
    SS_OPTION_EXCEPTION_FREE_CODE);
} /* end mdlInitializeSizes */


/* Function: mdlInitializeSampleTimes =========================================
 * Abstract:
 *    Initialize the sample times array.
 */
static void mdlInitializeSampleTimes(SimStruct *S)
{
    ssSetSampleTime(S, 0, INHERITED_SAMPLE_TIME);
    ssSetOffsetTime(S, 0, 0.0);
    ssSetModelReferenceSampleTimeDefaultInheritance(S);
} /* end mdlInitializeSampleTimes */

/* Function: mdlOutputs =======================================================
 * Abstract:
 *   Compute the outputs of the S-function.
 */
static void mdlOutputs(SimStruct *S, int_T tid)
{
    real_T            *y     = ssGetOutputPortRealSignal(S,0);
    const real_T      *p_basic_state_size     = mxGetPr(ssGetSFcnParam(S, 0));
    const real_T      *p_controls    = mxGetPr(ssGetSFcnParam(S, 1));
    int_T basic_state_size = p_basic_state_size[0];
    int_T controls = p_controls[0];
//
    int               iRow, iCol, width, iNum; 
    /*
     * Note1: Matrix signals are stored in column major order.
     * Note2: Access each matrix element by one index not two indices.
     *        For example, if the output signal is a [2x2] matrix signal,
     *        -          -
     *       | y[0]  y[2] |
     *       | y[1]  y[3] |
     *       -           -
     *       Output elements are stored as follows:
     *           y[0] --> row = 0, col = 0
     *           y[1] --> row = 1, col = 0
     *           y[2] --> row = 0, col = 1
     *           y[3] --> row = 1, col = 1
     */
    // matrix_a input port 0
    Eigen::MatrixXd matrix_a = Matrix::Zero(basic_state_size, basic_state_size);
    width = ssGetInputPortWidth(S, 0);
    InputRealPtrsType uPtrs_matrix_a = ssGetInputPortRealSignalPtrs(S,0);
    for(iNum = 0, iCol=0; iCol<basic_state_size; ++iCol){
        for(iRow=0; iRow<basic_state_size; ++iRow){
            if(iNum<width){
                matrix_a(iRow, iCol) = *uPtrs_matrix_a[iNum++];
            }
        }
    }
    // //matrix_b, basic_state_size, controls input port 1
    Eigen::MatrixXd matrix_b = Matrix::Zero(basic_state_size, controls);
    width = ssGetInputPortWidth(S, 1);
    InputRealPtrsType uPtrs_matrix_b = ssGetInputPortRealSignalPtrs(S,1);
    for(iNum=0, iCol=0; iCol<controls; ++iCol){
        for(iRow=0; iRow<basic_state_size; ++iRow){
            if(iNum<width){
                matrix_b(iRow, iCol) = *uPtrs_matrix_b[iNum++];
            }
        }
    }
    // //matrix_q, basic_state_size, basic_state_size input port 2
    Eigen::MatrixXd matrix_q = Matrix::Zero(basic_state_size, basic_state_size);
    width = ssGetInputPortWidth(S, 2);
    InputRealPtrsType uPtrs_matrix_q = ssGetInputPortRealSignalPtrs(S,2);
    for(iNum=0,iCol=0; iCol<basic_state_size; ++iCol){
        for(iRow=0; iRow<basic_state_size; ++iRow){
            if(iNum<width){
                matrix_q(iRow, iCol) = *uPtrs_matrix_q[iNum++];
            }
        }
    }
    // matrix_r, controls, controls input port 3
    Eigen::MatrixXd matrix_r = Matrix::Zero(controls, controls);
    width = ssGetInputPortWidth(S, 3);
    InputRealPtrsType uPtrs_matrix_r = ssGetInputPortRealSignalPtrs(S,3);
    for(iNum=0,iCol=0; iCol<controls; ++iCol){
        for(iRow=0; iRow<controls; ++iRow){
            if(iNum<width){
                matrix_r(iRow, iCol) = *uPtrs_matrix_r[iNum++];
            }
        }
    }
    // // //lower_bound, controls, 1 input port 4
    Eigen::MatrixXd lower_bound = Matrix::Zero(controls, 1);
    width = ssGetInputPortWidth(S, 4);
    InputRealPtrsType uPtrs_matrix_lb = ssGetInputPortRealSignalPtrs(S,4);
    for(iNum=0, iCol=0; iCol<1; ++iCol){
        for(iRow=0; iRow<controls; ++iRow){
            if(iNum<width){
                lower_bound(iRow, iCol) = *uPtrs_matrix_lb[iNum++];
            }
        }
    }
    // // //upper_bound, controls, 1 input port 5
    Eigen::MatrixXd upper_bound = Matrix::Zero(controls, 1);
    width = ssGetInputPortWidth(S, 5);
    InputRealPtrsType uPtrs_matrix_ub = ssGetInputPortRealSignalPtrs(S,5);
    for(iNum=0,iCol=0; iCol<1; ++iCol){
        for(iRow=0; iRow<controls; ++iRow){
            if(iNum<width){
                upper_bound(iRow, iCol) = *uPtrs_matrix_ub[iNum++];
            }
        }
    }
    // // //lower_state_bound, basic_state_size, 1 input port 6
    Eigen::MatrixXd lower_state_bound = Matrix::Zero(basic_state_size, 1);
    width = ssGetInputPortWidth(S,6);
    InputRealPtrsType uPtrs_matrix_lsb = ssGetInputPortRealSignalPtrs(S,6);
    for(iNum=0, iCol=0; iCol<1; ++iCol){
        for(iRow=0; iRow<basic_state_size; ++iRow){
            if(iNum<width){
                lower_state_bound(iRow, iCol) = *uPtrs_matrix_lsb[iNum++];
            }
        }
    }
    // // // //upper_state_bound, basic_state_size,1 input port 7
    Eigen::MatrixXd upper_state_bound = Matrix::Zero(basic_state_size, 1);
    width = ssGetInputPortWidth(S, 7);
    InputRealPtrsType uPtrs_matrix_usb = ssGetInputPortRealSignalPtrs(S,7);
    for(iNum=0, iCol=0; iCol<1; ++iCol){
        for(iRow=0; iRow<basic_state_size; ++iRow){
            if(iNum<width){
                upper_state_bound(iRow, iCol) = *uPtrs_matrix_usb[iNum++];
            }
        }
    }
    // // //reference_state, basic_state_size,1 input port 8
    Eigen::MatrixXd reference_state = Matrix::Zero(basic_state_size, 1);
    width = ssGetInputPortWidth(S, 8);
    InputRealPtrsType uPtrs_matrix_ref = ssGetInputPortRealSignalPtrs(S,8);
    for(iNum=0, iCol=0; iCol<1; ++iCol){
        for(iRow=0; iRow<basic_state_size; ++iRow){
            if(iNum<width){
                reference_state(iRow, iCol) = *uPtrs_matrix_ref[iNum++];
            }
        }
    }
    // // //reference_state, basic_state_size,1 input port 9
    Eigen::MatrixXd matrix_state = Matrix::Zero(basic_state_size, 1);
    width = ssGetInputPortWidth(S, 9);
    InputRealPtrsType uPtrs_matrix_state = ssGetInputPortRealSignalPtrs(S,9);
    for(iNum=0, iCol=0; iCol<1; ++iCol){
        for(iRow=0; iRow<basic_state_size; ++iRow){
            if(iNum<width){
                matrix_state(iRow, iCol) = *uPtrs_matrix_state[iNum++];
            }
        }
    }
    //horizon, 1 input port 10
    InputRealPtrsType uPtrs_matrix_horizon = ssGetInputPortRealSignalPtrs(S,10);
    int horizon = *uPtrs_matrix_horizon[0];
    //mpc_max_iteration, 1 input port 11
    InputRealPtrsType uPtrs_matrix_maxIter = ssGetInputPortRealSignalPtrs(S,11);
    int mpc_max_iteration = *uPtrs_matrix_maxIter[0];
    //mpc_eps, 1 input port 12
    InputRealPtrsType uPtrs_matrix_mpcEps = ssGetInputPortRealSignalPtrs(S,12);
    double mpc_eps = *uPtrs_matrix_mpcEps[0];
   
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
    // !!!!!!
    // need to be changed here
    // !!!!!!
    y[0] = 100;
    y[0] = basic_state_size;
    for(iRow=0; iRow<basic_state_size; ++iRow){
        y[0] += *uPtrs_matrix_a[iRow];
    }
    // y[0] = controls;
    
} /* end mdlOutputs */


#if defined(MATLAB_MEX_FILE)
#define MDL_SET_INPUT_PORT_DIMENSION_INFO
/* Function: mdlSetInputPortDimensionInfo ====================================
 * Abstract:
 *    This routine is called with the candidate dimensions for an input port
 *    with unknown dimensions. If the proposed dimensions are acceptable, the
 *    routine should go ahead and set the actual port dimensions.
 *    If they are unacceptable an error should be generated via
 *    ssSetErrorStatus.
 *    Note that any other input or output ports whose dimensions are
 *    implicitly defined by virtue of knowing the dimensions of the given port
 *    can also have their dimensions set.
 */
static void mdlSetInputPortDimensionInfo(SimStruct        *S,
int_T            port,
const DimsInfo_T *dimsInfo)
{
    const real_T      *p_basic_state_size     = mxGetPr(ssGetSFcnParam(S, 0));
    const real_T      *p_controls    = mxGetPr(ssGetSFcnParam(S, 1));
    int_T basic_state_size = p_basic_state_size[0];
    int_T controls = p_controls[0];


    int_T  uNumDims   = dimsInfo->numDims;
    int_T  uWidth     = dimsInfo->width;
    int32_T  *uDims   = dimsInfo->dims;
    
    int_T numDims = 0;
    boolean_T  isOk = true;
    int_T iParam = 0;
    int_T outWidth = ssGetOutputPortWidth(S, 0);
    
    // input0, matrix_a, basic_state_size, basic_state_size
    DECL_AND_INIT_DIMSINFO(di0);
    int_T dims0[2];
    di0.numDims = 2;
    dims0[0] = basic_state_size;
    dims0[1] = basic_state_size;
    di0.dims = dims0;
    di0.width = basic_state_size*basic_state_size;
    /* Set input port dimension */
    if(!ssSetInputPortDimensionInfo(S, 0, &di0)) return;
    
    // // input1, matrix_b, basic_state_size, controls
    DECL_AND_INIT_DIMSINFO(di1);
    int_T dims1[2];
    di1.numDims = 2;
    dims1[0] = basic_state_size;
    dims1[1] = controls;
    di1.dims = dims1;
    di1.width = basic_state_size*controls;
    /* Set input port dimension */
    if(!ssSetInputPortDimensionInfo(S, 1, &di1)) return;

    //input2, matrix_q, basic_state_size, basic_state_size
    DECL_AND_INIT_DIMSINFO(di2);
    int_T dims2[2];
    di2.numDims = 2;
    dims2[0] = basic_state_size;
    dims2[1] = basic_state_size;
    di2.dims = dims2;
    di2.width = basic_state_size*basic_state_size;
    /* Set input port dimension */
    if(!ssSetInputPortDimensionInfo(S, 2, &di2)) return;

    //input3, matrix_r, controls, controls
    DECL_AND_INIT_DIMSINFO(di3);
    int_T dims3[2];
    di3.numDims = 2;
    dims3[0] = controls;
    dims3[1] = controls;
    di3.dims = dims3;
    di3.width = controls*controls;
    /* Set input port dimension */
    if(!ssSetInputPortDimensionInfo(S, 3, &di3)) return;

    //input4, lower_bound, controls, 1
    DECL_AND_INIT_DIMSINFO(di4);
    int_T dims4[2];
    di4.numDims = 2;
    dims4[0] = controls;
    dims4[1] = 1;
    di4.dims = dims4;
    di4.width = controls*1;
    /* Set input port dimension */
    if(!ssSetInputPortDimensionInfo(S, 4, &di4)) return;

    //input5, upper_bound, controls, 1
    DECL_AND_INIT_DIMSINFO(di5);
    int_T dims5[2];
    di5.numDims = 2;
    dims5[0] = controls;
    dims5[1] = 1;
    di5.dims = dims5;
    di5.width = controls*1;
    /* Set input port dimension */
    if(!ssSetInputPortDimensionInfo(S, 5, &di5)) return;

    //input6, lower_state_bound, basic_state_size, 1
    DECL_AND_INIT_DIMSINFO(di6);
    int_T dims6[2];
    di6.numDims = 2;
    dims6[0] = basic_state_size;
    dims6[1] = 1;
    di6.dims = dims6;
    di6.width = basic_state_size*1;
    /* Set input port dimension */
    if(!ssSetInputPortDimensionInfo(S, 6, &di6)) return;

    //input7, upper_state_bound, basic_state_size, 1
    DECL_AND_INIT_DIMSINFO(di7);
    int_T dims7[2];
    di7.numDims = 2;
    dims7[0] = basic_state_size;
    dims7[1] = 1;
    di7.dims = dims7;
    di7.width = basic_state_size*1;
    /* Set input port dimension */
    if(!ssSetInputPortDimensionInfo(S, 7, &di7)) return;

    //input8, reference_state, basic_state_size,1
    DECL_AND_INIT_DIMSINFO(di8);
    int_T dims8[2];
    di8.numDims = 2;
    dims8[0] = basic_state_size;
    dims8[1] = 1;
    di8.dims = dims8;
    di8.width = basic_state_size*1;
    /* Set input port dimension */
    if(!ssSetInputPortDimensionInfo(S, 8, &di8)) return;

    //input9, init_state, basic_state_size,1
    DECL_AND_INIT_DIMSINFO(di9);
    int_T dims9[2];
    di9.numDims = 2;
    dims9[0] = basic_state_size;
    dims9[1] = 1;
    di9.dims = dims9;
    di9.width = basic_state_size*1;
    /* Set input port dimension */
    if(!ssSetInputPortDimensionInfo(S, 9, &di9)) return;

    //input10, horizon, 1,1
    DECL_AND_INIT_DIMSINFO(di10);
    int_T dims10[2];
    di10.numDims = 2;
    dims10[0] = 1;
    dims10[1] = 1;
    di10.dims = dims10;
    di10.width = 1*1;
    /* Set input port dimension */
    if(!ssSetInputPortDimensionInfo(S, 10, &di10)) return;

    //input11, mpc_max_iteration, 1,1
    DECL_AND_INIT_DIMSINFO(di11);
    int_T dims11[2];
    di11.numDims = 2;
    dims11[0] = 1;
    dims11[1] = 1;
    di11.dims = dims11;
    di11.width = 1*1;
    /* Set input port dimension */
    if(!ssSetInputPortDimensionInfo(S, 11, &di11)) return;

    //input12, eps, 1,1
    DECL_AND_INIT_DIMSINFO(di12);
    int_T dims12[2];
    di12.numDims = 2;
    dims12[0] = 1;
    dims12[1] = 1;
    di12.dims = dims12;
    di12.width = 1*1;
    /* Set input port dimension */
    if(!ssSetInputPortDimensionInfo(S, 12, &di12)) return;
} /* end mdlSetInputPortDimensionInfo */

# define MDL_SET_OUTPUT_PORT_DIMENSION_INFO
/* Function: mdlSetOutputPortDimensionInfo ===================================
 * Abstract:
 *    This routine is called with the candidate dimensions for an output port
 *    with unknown dimensions. If the proposed dimensions are acceptable, the
 *    routine should go ahead and set the actual port dimensions.
 *    If they are unacceptable an error should be generated via
 *    ssSetErrorStatus.
 *    Note that any other input or output ports whose dimensions are
 *    implicitly defined by virtue of knowing the dimensions of the given
 *    port can also have their dimensions set.
 */
static void mdlSetOutputPortDimensionInfo(SimStruct        *S,
int_T            port,
const DimsInfo_T *dimsInfo)
{
    /*
     * If the block has scalar parameter, the output dimensions are unknown.
     * Set the input and output port to have the same dimensions.
     */
    if(!ssSetOutputPortDimensionInfo(S, port, dimsInfo)) return;
    
    /* The block only accepts 2-D or n-D signals. Check number of dimensions. */
    if (!(dimsInfo->numDims >= 2)){
        ssSetErrorStatus(S, "Invalid output port dimensions. The output signal "
        "must be a 2-D or n-D array (matrix) signal.");
        return;
    }else{
        /* Set the input port dimensions */
        if(!ssSetInputPortDimensionInfo(S, port, dimsInfo)) return;
    }
} /* end mdlSetOutputPortDimensionInfo */

# define MDL_SET_DEFAULT_PORT_DIMENSION_INFO
/* Function: mdlSetDefaultPortDimensionInfo ====================================
 *    This routine is called when Simulink is not able to find dimension
 *    candidates for ports with unknown dimensions. This function must set the
 *    dimensions of all ports with unknown dimensions.
 */
static void mdlSetDefaultPortDimensionInfo(SimStruct *S)
{
    int_T outWidth = ssGetOutputPortWidth(S, 0);
    /* Input port dimension must be unknown. Set it to scalar. */
    if(!ssSetInputPortMatrixDimensions(S, 0, 1, 1)) return;
    if(outWidth == DYNAMICALLY_SIZED){
        /* Output dimensions are unknown. Set it to scalar. */
        if(!ssSetOutputPortMatrixDimensions(S, 0, 1, 1)) return;
    }
} /* end mdlSetDefaultPortDimensionInfo */
#endif


/* Function: mdlTerminate =====================================================
 * Abstract:
 *    Called when the simulation is terminated.
 */
static void mdlTerminate(SimStruct *S)
{
    /* Deallocate intdimensions stored in user data*/
    // int32_T *dims = ssGetUserData(S);
    // free(dims);
    
} /* end mdlTerminate */

#ifdef	MATLAB_MEX_FILE
#include "simulink.c"
#else
#include "cg_sfun.h"
#endif

/* [EOF] sfun_matadd.c */
