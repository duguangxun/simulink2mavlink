#define S_FUNCTION_NAME  mavlink_msg_gen
#define S_FUNCTION_LEVEL 2

#include "simstruc.h"
#include "mavlink_msg_gen.h"

// // message ID
// #define MSG_ID_IDX 0
// #define MSG_ID(S) ssGetSFcnParam(S,MSG_ID_IDX)
// //Total number of parameters
// #define NPARAMS   1

/* Error handling
 * --------------
 *
 * You should use the following technique to report errors encountered within
 * an S-function:
 *
 *       ssSetErrorStatus(S,"Error encountered due to ...");
 *       return;
 *
 * Note that the 2nd argument to ssSetErrorStatus must be persistent memory.
 * It cannot be a local variable. For example the following will cause
 * unpredictable errors:
 *
 *      mdlOutputs()
 *      {
 *         char msg[256];         {ILLEGAL: to fix use "static char msg[256];"}
 *         sprintf(msg,"Error due to %s", string);
 *         ssSetErrorStatus(S,msg);
 *         return;
 *      }
 *
 * See matlabroot/simulink/src/sfuntmpl_doc.c for more details.
 */

/*====================*
 * S-function methods *
 *====================*/

/* Function: mdlInitializeSizes ===============================================
 * Abstract:
 *    The sizes information is used by Simulink to determine the S-function
 *    block's characteristics (number of inputs, outputs, states, etc.).
 */
static void mdlInitializeSizes(SimStruct *S)
{
    /* See sfuntmpl_doc.c for more details on the macros below */

    ssSetNumSFcnParams(S, 0);  /* Number of expected parameters */
    if (ssGetNumSFcnParams(S) != ssGetSFcnParamsCount(S)) {
        /* Return if number of expected != number of actual parameters */
        return;
    }

    ssSetNumContStates(S, 0);
    ssSetNumDiscStates(S, 0);

    if (!ssSetNumInputPorts(S, 1)) return;
    ssSetInputPortWidth(S, 0, 34);
    ssSetInputPortRequiredContiguous(S, 0, true); /*direct input signal access*/
    /*
     * Set direct feedthrough flag (1=yes, 0=no).
     * A port has direct feedthrough if the input is used in either
     * the mdlOutputs or mdlGetTimeOfNextVarHit functions.
     * See matlabroot/simulink/src/sfuntmpl_directfeed.txt.
     */
    ssSetInputPortDirectFeedThrough(S, 0, 1);

    if (!ssSetNumOutputPorts(S, 1)) return;
    ssSetOutputPortWidth(S, 0, MAVLINK_MAX_PACKET_LEN);
//     ssSetOutputPortWidth(S, 0, mask_msg_id);

    ssSetNumSampleTimes(S, 1);
    ssSetNumRWork(S, 0);
    ssSetNumIWork(S, 0);
    ssSetNumPWork(S, 0);
    ssSetNumModes(S, 0);
    ssSetNumNonsampledZCs(S, 0);

    /* Specify the sim state compliance to be same as a built-in block */
    ssSetSimStateCompliance(S, USE_DEFAULT_SIM_STATE);

    ssSetOptions(S, 0);
//     ssSetNumSFcnParams(S, NPARAMS);  /* Number of expected parameters */
//     ssSetSFcnParamTunable(S,MSG_ID_IDX,false);
}



/* Function: mdlInitializeSampleTimes =========================================
 * Abstract:
 *    This function is used to specify the sample time(s) for your
 *    S-function. You must register the same number of sample times as
 *    specified in ssSetNumSampleTimes.
 */
static void mdlInitializeSampleTimes(SimStruct *S)
{
    ssSetSampleTime(S, 0, INHERITED_SAMPLE_TIME);
    ssSetOffsetTime(S, 0, 0.0);

}



#define MDL_INITIALIZE_CONDITIONS   /* Change to #undef to remove function */
#if defined(MDL_INITIALIZE_CONDITIONS)
  /* Function: mdlInitializeConditions ========================================
   * Abstract:
   *    In this function, you should initialize the continuous and discrete
   *    states for your S-function block.  The initial states are placed
   *    in the state vector, ssGetContStates(S) or ssGetRealDiscStates(S).
   *    You can also perform any other initialization activities that your
   *    S-function may require. Note, this routine will be called at the
   *    start of simulation and if it is present in an enabled subsystem
   *    configured to reset states, it will be call when the enabled subsystem
   *    restarts execution to reset the states.
   */
  static void mdlInitializeConditions(SimStruct *S)
  {
  }
#endif /* MDL_INITIALIZE_CONDITIONS */



#define MDL_START  /* Change to #undef to remove function */
#if defined(MDL_START) 
  /* Function: mdlStart =======================================================
   * Abstract:
   *    This function is called once at start of model execution. If you
   *    have states that should be initialized once, this is the place
   *    to do it.
   */
  static void mdlStart(SimStruct *S)
  {
  }
#endif /*  MDL_START */



/* Function: mdlOutputs =======================================================
 * Abstract:
 *    In this function, you compute the outputs of your S-function
 *    block.
 */
static void mdlOutputs(SimStruct *S, int_T tid)
{  
    uint16_t i;
    const real_T *uPtr = (const real_T*) ssGetInputPortSignal(S,0);
    real_T *y = ssGetOutputPortRealSignal(S, 0);
    uint64_T time_usecs=(uint64_T)uPtr[0];
    real_T lat=uPtr[1];
    real_T lon = uPtr[2]; 
    real_T alt = uPtr[3];
    uint8_T fix_type = (uint8_T)uPtr[4]; 
    real_T gnd_vel = uPtr[5];
    real_T v_n = uPtr[6];
    real_T v_e = uPtr[7];
    real_T v_d = uPtr[8];
    real_T cog = uPtr[9];
    real_T ax = uPtr[10];
    real_T ay = uPtr[11];
    real_T az = uPtr[12];
    real_T p = uPtr[13];
    real_T q = uPtr[14];
    real_T r = uPtr[15];
    real_T mx = uPtr[16];
    real_T my = uPtr[17];
    real_T mz = uPtr[18];
    real_T abs_pressure = uPtr[19];
    real_T diff_pressure = uPtr[20];
    real_T temperature = uPtr[21];
    real_T u = uPtr[22];
    real_T v = uPtr[23];
    real_T w = uPtr[24];
    real_T q1 = uPtr[25];
    real_T q2 = uPtr[26];
    real_T q3 = uPtr[27];
    real_T q4 = uPtr[28];
    real_T phi = uPtr[29];
    real_T theta = uPtr[30];
    real_T psi = uPtr[31];
    uint8_T msg_id = (uint8_T)uPtr[32];
    uint8_T target_sys_id = (uint8_T)uPtr[33];
    
    real_T alpha = atan(w/u);
    real_T wind_mag = sqrt(u*u + v*v + w*w);
    real_T beta = asin(v/wind_mag);
    int32_T lat_int = (int32_T)lat*1000000;
    int32_T lon_int = (int32_T)lon*1000000;
    int32_T alt_int = (int32_T)alt*1000000;
    real_T rho0 = 1013.25;
    //http://www.srh.noaa.gov/images/epz/wxcalc/pressureAltitude.pdf
    real_T pressure_alt = (1-pow((abs_pressure/rho0),0.190284))*145366.45*0.3048;
          
    switch(msg_id){
        case MAVLINK_MSG_ID_HEARTBEAT: //mavlink_msg_heartbeat_pack(mavlink_system.sysid,MAV_COMP_ID_SYSTEM_CONTROL,&out_msg_ml, MAV_TYPE_FIXED_WING, MAV_AUTOPILOT_GENERIC, MAV_MODE_AUTO_ARMED, MAV_MODE_FLAG_MANUAL_INPUT_ENABLED, MAV_STATE_ACTIVE);
            mavlink_msg_heartbeat_pack(mavlink_system.sysid, mavlink_system.compid,&out_msg_ml,mavlink_system.type, mavlink_system.autopilot, mavlink_system.mode,mavlink_system.custom_mode, mavlink_system.state);
            break;
        //Need to get the sys_id of the target system from mavlink. Current need to give it as an input
        case MAVLINK_MSG_ID_COMMAND_LONG:
            //this message just sets the mode to HILS
            mavlink_msg_command_long_pack(mavlink_system.sysid, mavlink_system.compid, &out_msg_ml, target_sys_id, MAV_COMP_ID_ALL, MAV_CMD_DO_SET_MODE, 4, MAV_MODE_FLAG_AUTO_ENABLED | MAV_MODE_FLAG_SAFETY_ARMED | MAV_MODE_FLAG_HIL_ENABLED, 0, 0, 0, 0, 0, 0);
//             mavlink_msg_command_long_pack(mavlink_system.sysid, mavlink_system.compid, &out_msg_ml, target_sys_id, MAV_COMP_ID_ALL, MAV_CMD_DO_SET_MODE, 4, MAV_MODE_FLAG_AUTO_ENABLED | MAV_MODE_FLAG_SAFETY_ARMED | MAV_MODE_FLAG_HIL_ENABLED, 0, 0, 0, 0, 0, 0);
            break;//msg_id:76
        case MAVLINK_MSG_ID_HIL_SENSOR:            
            mavlink_msg_hil_sensor_pack(mavlink_system.sysid, mavlink_system.compid, &out_msg_ml,time_usecs, ax, ay, az, p, q, r, mx, my, mz, abs_pressure, diff_pressure, pressure_alt, temperature, 4095);
            break; //msg_id:107
        case MAVLINK_MSG_ID_HIL_GPS: 
             mavlink_msg_hil_gps_pack(mavlink_system.sysid, mavlink_system.compid, &out_msg_ml,time_usecs, fix_type, lat_int, lon_int, alt_int, 65535, 65535, (uint16_T) gnd_vel*100, (int16_T) v_n*100, (int16_T) v_e*100, (int16_T) v_d*100, (uint16_T) cog*100, 255);
            break;  //msg_id:113
        case MAVLINK_MSG_ID_HIL_STATE_QUATERNION: 
//             http://www.princeton.edu/~stengel/MAE331Lecture9.pdf
//             real_T phi_dot = p + (sin(phi)*tan(theta)*q) + (cos(phi)*tan(theta)*r);
//             real_T theta_dot = cos(phi)*q + (-sin(phi)*r);
//             real_T psi_dot = ((sin(phi)/cos(theta))*q) + ((cos(phi)/cos(theta))*r);
//             https://pixhawk.ethz.ch/mavlink/#HIL_STATE_QUATERNION
            real_T phi_dot = p;
            real_T theta_dot = q;
            real_T psi_dot = r;
            uint16_T ind_airspeed = (uint16_T) sqrt((2*diff_pressure)/rho0)*100;
            uint16_T true_airspeed = (uint16_T) sqrt((2*diff_pressure)/abs_pressure)*100;
            float Quat[4]={q1,q2,q3,q4};
            mavlink_msg_hil_state_quaternion_pack(mavlink_system.sysid, mavlink_system.compid, &out_msg_ml,time_usecs, Quat, phi_dot, theta_dot, psi_dot, lat_int, lon_int, alt_int, (int16_t) u*100, (int16_t) v*100, (int16_t) w*100, ind_airspeed, true_airspeed, (int16_t) ax*100, (int16_t) ay*100, (int16_t) az*100);
            break; //msg_id:115
    }
    static uint8_T* out_msg = new uint8_T[MavlinkMessageSizes[(uint8_t)msg_id]+MAVLINK_NUM_NON_PAYLOAD_BYTES];
    mavlink_msg_to_send_buffer(out_msg,&out_msg_ml);
    for(i=0;i<MavlinkMessageSizes[(uint16_T)msg_id]+MAVLINK_NUM_NON_PAYLOAD_BYTES;i++){  y[i]=out_msg[i];  ssPrintf("%x ",out_msg[i]);}ssPrintf("\n");
//     ssPrintf("\n%d\n",MSG_ID(S));
//      y[0]=MavlinkMessageSizes[(uint16_T)msg_id];
//      ssPrintf("%d %d\n",(uPtr[29]),(uPtr[30]));
}



#define MDL_UPDATE  /* Change to #undef to remove function */
#if defined(MDL_UPDATE)
  /* Function: mdlUpdate ======================================================
   * Abstract:
   *    This function is called once for every major integration time step.
   *    Discrete states are typically updated here, but this function is useful
   *    for performing any tasks that should only take place once per
   *    integration step.
   */
  static void mdlUpdate(SimStruct *S, int_T tid)
  {
  }
#endif /* MDL_UPDATE */



#define MDL_DERIVATIVES  /* Change to #undef to remove function */
#if defined(MDL_DERIVATIVES)
  /* Function: mdlDerivatives =================================================
   * Abstract:
   *    In this function, you compute the S-function block's derivatives.
   *    The derivatives are placed in the derivative vector, ssGetdX(S).
   */
  static void mdlDerivatives(SimStruct *S)
  {
  }
#endif /* MDL_DERIVATIVES */



/* Function: mdlTerminate =====================================================
 * Abstract:
 *    In this function, you should perform any actions that are necessary
 *    at the termination of a simulation.  For example, if memory was
 *    allocated in mdlStart, this is the place to free it.
 */
static void mdlTerminate(SimStruct *S)
{
//     delete[]out_msg;
}


/*======================================================*
 * See sfuntmpl_doc.c for the optional S-function methods *
 *======================================================*/

/*=============================*
 * Required S-function trailer *
 *=============================*/

#ifdef  MATLAB_MEX_FILE    /* Is this file being compiled as a MEX-file? */
#include "simulink.c"      /* MEX-file interface mechanism */
#else
#include "cg_sfun.h"       /* Code generation registration function */
#endif
