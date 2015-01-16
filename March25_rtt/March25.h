/*
 * File: March25.h
 *
 * Code generated for Simulink model 'March25'.
 *
 * Model version                  : 1.8
 * Simulink Coder version         : 8.6 (R2014a) 27-Dec-2013
 * TLC version                    : 8.6 (Jan 30 2014)
 * C/C++ source code generated on : Thu Apr 03 12:41:32 2014
 *
 * Target selection: realtime.tlc
 * Embedded hardware selection: Atmel->AVR
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#ifndef RTW_HEADER_March25_h_
#define RTW_HEADER_March25_h_
#include <float.h>
#include <string.h>
#include <stddef.h>
#ifndef March25_COMMON_INCLUDES_
# define March25_COMMON_INCLUDES_
#include "rtwtypes.h"
#include "rtw_extmode.h"
#include "sysran_types.h"
#include "rtw_continuous.h"
#include "rtw_solver.h"
#include "dt_info.h"
#include "ext_work.h"
#include "arduino_digitaloutput_lct.h"
#endif                                 /* March25_COMMON_INCLUDES_ */

#include "March25_types.h"

/* Shared type includes */
#include "multiword_types.h"

/* Macros for accessing real-time model data structure */
#ifndef rtmGetFinalTime
# define rtmGetFinalTime(rtm)          ((rtm)->Timing.tFinal)
#endif

#ifndef rtmGetErrorStatus
# define rtmGetErrorStatus(rtm)        ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
# define rtmSetErrorStatus(rtm, val)   ((rtm)->errorStatus = (val))
#endif

#ifndef rtmGetStopRequested
# define rtmGetStopRequested(rtm)      ((rtm)->Timing.stopRequestedFlag)
#endif

#ifndef rtmSetStopRequested
# define rtmSetStopRequested(rtm, val) ((rtm)->Timing.stopRequestedFlag = (val))
#endif

#ifndef rtmGetStopRequestedPtr
# define rtmGetStopRequestedPtr(rtm)   (&((rtm)->Timing.stopRequestedFlag))
#endif

#ifndef rtmGetT
# define rtmGetT(rtm)                  ((rtm)->Timing.taskTime0)
#endif

#ifndef rtmGetTFinal
# define rtmGetTFinal(rtm)             ((rtm)->Timing.tFinal)
#endif

/* Block states (auto storage) for system '<Root>' */
typedef struct {
  int32_T clockTickCounter;            /* '<Root>/Pulse Generator2' */
} D_Work_March25;

/* Parameters (auto storage) */
struct Parameters_March25_ {
  uint32_T DigitalOutput_pinNumber;    /* Mask Parameter: DigitalOutput_pinNumber
                                        * Referenced by: '<S4>/Digital Output'
                                        */
  uint32_T DigitalOutput_pinNumber_b;  /* Mask Parameter: DigitalOutput_pinNumber_b
                                        * Referenced by: '<S5>/Digital Output'
                                        */
  uint32_T DigitalOutput_pinNumber_k;  /* Mask Parameter: DigitalOutput_pinNumber_k
                                        * Referenced by: '<S1>/Digital Output'
                                        */
  uint32_T DigitalOutput_pinNumber_e;  /* Mask Parameter: DigitalOutput_pinNumber_e
                                        * Referenced by: '<S3>/Digital Output'
                                        */
  uint32_T DigitalOutput_pinNumber_c;  /* Mask Parameter: DigitalOutput_pinNumber_c
                                        * Referenced by: '<S7>/Digital Output'
                                        */
  real_T Constant1_Value;              /* Expression: 1
                                        * Referenced by: '<Root>/Constant1'
                                        */
  real_T PulseGenerator2_Amp;          /* Expression: 1
                                        * Referenced by: '<Root>/Pulse Generator2'
                                        */
  real_T PulseGenerator2_Period;       /* Expression: 2
                                        * Referenced by: '<Root>/Pulse Generator2'
                                        */
  real_T PulseGenerator2_Duty;         /* Expression: 1
                                        * Referenced by: '<Root>/Pulse Generator2'
                                        */
  real_T PulseGenerator2_PhaseDelay;   /* Expression: 0
                                        * Referenced by: '<Root>/Pulse Generator2'
                                        */
};

/* Real-time Model Data Structure */
struct tag_RTM_March25 {
  const char_T *errorStatus;
  RTWExtModeInfo *extModeInfo;

  /*
   * Sizes:
   * The following substructure contains sizes information
   * for many of the model attributes such as inputs, outputs,
   * dwork, sample times, etc.
   */
  struct {
    uint32_T checksums[4];
  } Sizes;

  /*
   * SpecialInfo:
   * The following substructure contains special information
   * related to other components that are dependent on RTW.
   */
  struct {
    const void *mappingInfo;
  } SpecialInfo;

  /*
   * Timing:
   * The following substructure contains information regarding
   * the timing information for the model.
   */
  struct {
    time_T taskTime0;
    uint32_T clockTick0;
    uint32_T clockTickH0;
    time_T stepSize0;
    time_T tFinal;
    boolean_T stopRequestedFlag;
  } Timing;
};

/* Block parameters (auto storage) */
extern Parameters_March25 March25_P;

/* Block states (auto storage) */
extern D_Work_March25 March25_DWork;

/* Model entry point functions */
extern void March25_initialize(void);
extern void March25_output(void);
extern void March25_update(void);
extern void March25_terminate(void);

/* Real-time Model object */
extern RT_MODEL_March25 *const March25_M;

/*-
 * The generated code includes comments that allow you to trace directly
 * back to the appropriate location in the model.  The basic format
 * is <system>/block_name, where system is the system number (uniquely
 * assigned by Simulink) and block_name is the name of the block.
 *
 * Use the MATLAB hilite_system command to trace the generated code back
 * to the model.  For example,
 *
 * hilite_system('<S3>')    - opens system 3
 * hilite_system('<S3>/Kp') - opens and selects block Kp which resides in S3
 *
 * Here is the system hierarchy for this model
 *
 * '<Root>' : 'March25'
 * '<S1>'   : 'March25/Digital Output'
 * '<S2>'   : 'March25/Digital Output1'
 * '<S3>'   : 'March25/Digital Output3'
 * '<S4>'   : 'March25/Digital Output5'
 * '<S5>'   : 'March25/Direction'
 * '<S6>'   : 'March25/Enable'
 * '<S7>'   : 'March25/Step'
 */
#endif                                 /* RTW_HEADER_March25_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
