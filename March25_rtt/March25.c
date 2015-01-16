/*
 * File: March25.c
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

#include "March25.h"
#include "March25_private.h"
#include "March25_dt.h"

/* Block states (auto storage) */
D_Work_March25 March25_DWork;

/* Real-time model */
RT_MODEL_March25 March25_M_;
RT_MODEL_March25 *const March25_M = &March25_M_;

/* Model output function */
void March25_output(void)
{
  real_T rtb_PulseGenerator2;
  uint8_T rtb_PulseGenerator2_0;

  /* DataTypeConversion: '<S4>/Data Type Conversion' incorporates:
   *  Constant: '<Root>/Constant1'
   */
  if (March25_P.Constant1_Value < 256.0) {
    if (March25_P.Constant1_Value >= 0.0) {
      rtb_PulseGenerator2_0 = (uint8_T)March25_P.Constant1_Value;
    } else {
      rtb_PulseGenerator2_0 = 0U;
    }
  } else {
    rtb_PulseGenerator2_0 = MAX_uint8_T;
  }

  /* End of DataTypeConversion: '<S4>/Data Type Conversion' */

  /* S-Function (arduinodigitaloutput_sfcn): '<S4>/Digital Output' */
  MW_digitalWrite(March25_P.DigitalOutput_pinNumber, rtb_PulseGenerator2_0);

  /* DataTypeConversion: '<S5>/Data Type Conversion' incorporates:
   *  Constant: '<Root>/Constant1'
   */
  if (March25_P.Constant1_Value < 256.0) {
    if (March25_P.Constant1_Value >= 0.0) {
      rtb_PulseGenerator2_0 = (uint8_T)March25_P.Constant1_Value;
    } else {
      rtb_PulseGenerator2_0 = 0U;
    }
  } else {
    rtb_PulseGenerator2_0 = MAX_uint8_T;
  }

  /* End of DataTypeConversion: '<S5>/Data Type Conversion' */

  /* S-Function (arduinodigitaloutput_sfcn): '<S5>/Digital Output' */
  MW_digitalWrite(March25_P.DigitalOutput_pinNumber_b, rtb_PulseGenerator2_0);

  /* DiscretePulseGenerator: '<Root>/Pulse Generator2' */
  rtb_PulseGenerator2 = (March25_DWork.clockTickCounter <
    March25_P.PulseGenerator2_Duty) && (March25_DWork.clockTickCounter >= 0L) ?
    March25_P.PulseGenerator2_Amp : 0.0;
  if (March25_DWork.clockTickCounter >= March25_P.PulseGenerator2_Period - 1.0)
  {
    March25_DWork.clockTickCounter = 0L;
  } else {
    March25_DWork.clockTickCounter++;
  }

  /* End of DiscretePulseGenerator: '<Root>/Pulse Generator2' */

  /* DataTypeConversion: '<S1>/Data Type Conversion' */
  if (rtb_PulseGenerator2 < 256.0) {
    if (rtb_PulseGenerator2 >= 0.0) {
      rtb_PulseGenerator2_0 = (uint8_T)rtb_PulseGenerator2;
    } else {
      rtb_PulseGenerator2_0 = 0U;
    }
  } else {
    rtb_PulseGenerator2_0 = MAX_uint8_T;
  }

  /* End of DataTypeConversion: '<S1>/Data Type Conversion' */

  /* S-Function (arduinodigitaloutput_sfcn): '<S1>/Digital Output' */
  MW_digitalWrite(March25_P.DigitalOutput_pinNumber_k, rtb_PulseGenerator2_0);

  /* DataTypeConversion: '<S3>/Data Type Conversion' */
  if (rtb_PulseGenerator2 < 256.0) {
    if (rtb_PulseGenerator2 >= 0.0) {
      rtb_PulseGenerator2_0 = (uint8_T)rtb_PulseGenerator2;
    } else {
      rtb_PulseGenerator2_0 = 0U;
    }
  } else {
    rtb_PulseGenerator2_0 = MAX_uint8_T;
  }

  /* End of DataTypeConversion: '<S3>/Data Type Conversion' */

  /* S-Function (arduinodigitaloutput_sfcn): '<S3>/Digital Output' */
  MW_digitalWrite(March25_P.DigitalOutput_pinNumber_e, rtb_PulseGenerator2_0);

  /* DataTypeConversion: '<S7>/Data Type Conversion' */
  if (rtb_PulseGenerator2 < 256.0) {
    if (rtb_PulseGenerator2 >= 0.0) {
      rtb_PulseGenerator2_0 = (uint8_T)rtb_PulseGenerator2;
    } else {
      rtb_PulseGenerator2_0 = 0U;
    }
  } else {
    rtb_PulseGenerator2_0 = MAX_uint8_T;
  }

  /* End of DataTypeConversion: '<S7>/Data Type Conversion' */

  /* S-Function (arduinodigitaloutput_sfcn): '<S7>/Digital Output' */
  MW_digitalWrite(March25_P.DigitalOutput_pinNumber_c, rtb_PulseGenerator2_0);
}

/* Model update function */
void March25_update(void)
{
  /* signal main to stop simulation */
  {                                    /* Sample time: [1.0E-5s, 0.0s] */
    if ((rtmGetTFinal(March25_M)!=-1) &&
        !((rtmGetTFinal(March25_M)-March25_M->Timing.taskTime0) >
          March25_M->Timing.taskTime0 * (DBL_EPSILON))) {
      rtmSetErrorStatus(March25_M, "Simulation finished");
    }

    if (rtmGetStopRequested(March25_M)) {
      rtmSetErrorStatus(March25_M, "Simulation finished");
    }
  }

  /* Update absolute time for base rate */
  /* The "clockTick0" counts the number of times the code of this task has
   * been executed. The absolute time is the multiplication of "clockTick0"
   * and "Timing.stepSize0". Size of "clockTick0" ensures timer will not
   * overflow during the application lifespan selected.
   * Timer of this task consists of two 32 bit unsigned integers.
   * The two integers represent the low bits Timing.clockTick0 and the high bits
   * Timing.clockTickH0. When the low bit overflows to 0, the high bits increment.
   */
  if (!(++March25_M->Timing.clockTick0)) {
    ++March25_M->Timing.clockTickH0;
  }

  March25_M->Timing.taskTime0 = March25_M->Timing.clockTick0 *
    March25_M->Timing.stepSize0 + March25_M->Timing.clockTickH0 *
    March25_M->Timing.stepSize0 * 4294967296.0;
}

/* Model initialize function */
void March25_initialize(void)
{
  /* Registration code */

  /* initialize real-time model */
  (void) memset((void *)March25_M, 0,
                sizeof(RT_MODEL_March25));
  rtmSetTFinal(March25_M, -1);
  March25_M->Timing.stepSize0 = 1.0E-5;

  /* External mode info */
  March25_M->Sizes.checksums[0] = (2957236228U);
  March25_M->Sizes.checksums[1] = (49200937U);
  March25_M->Sizes.checksums[2] = (623645098U);
  March25_M->Sizes.checksums[3] = (2591176413U);

  {
    static const sysRanDType rtAlwaysEnabled = SUBSYS_RAN_BC_ENABLE;
    static RTWExtModeInfo rt_ExtModeInfo;
    static const sysRanDType *systemRan[1];
    March25_M->extModeInfo = (&rt_ExtModeInfo);
    rteiSetSubSystemActiveVectorAddresses(&rt_ExtModeInfo, systemRan);
    systemRan[0] = &rtAlwaysEnabled;
    rteiSetModelMappingInfoPtr(March25_M->extModeInfo,
      &March25_M->SpecialInfo.mappingInfo);
    rteiSetChecksumsPtr(March25_M->extModeInfo, March25_M->Sizes.checksums);
    rteiSetTPtr(March25_M->extModeInfo, rtmGetTPtr(March25_M));
  }

  /* states (dwork) */
  (void) memset((void *)&March25_DWork, 0,
                sizeof(D_Work_March25));

  /* data type transition information */
  {
    static DataTypeTransInfo dtInfo;
    (void) memset((char_T *) &dtInfo, 0,
                  sizeof(dtInfo));
    March25_M->SpecialInfo.mappingInfo = (&dtInfo);
    dtInfo.numDataTypes = 14;
    dtInfo.dataTypeSizes = &rtDataTypeSizes[0];
    dtInfo.dataTypeNames = &rtDataTypeNames[0];

    /* Block I/O transition table */
    dtInfo.B = &rtBTransTable;

    /* Parameters transition table */
    dtInfo.P = &rtPTransTable;
  }

  /* Start for S-Function (arduinodigitaloutput_sfcn): '<S4>/Digital Output' */
  MW_pinModeOutput(March25_P.DigitalOutput_pinNumber);

  /* Start for S-Function (arduinodigitaloutput_sfcn): '<S5>/Digital Output' */
  MW_pinModeOutput(March25_P.DigitalOutput_pinNumber_b);

  /* Start for DiscretePulseGenerator: '<Root>/Pulse Generator2' */
  March25_DWork.clockTickCounter = 0L;

  /* Start for S-Function (arduinodigitaloutput_sfcn): '<S1>/Digital Output' */
  MW_pinModeOutput(March25_P.DigitalOutput_pinNumber_k);

  /* Start for S-Function (arduinodigitaloutput_sfcn): '<S3>/Digital Output' */
  MW_pinModeOutput(March25_P.DigitalOutput_pinNumber_e);

  /* Start for S-Function (arduinodigitaloutput_sfcn): '<S7>/Digital Output' */
  MW_pinModeOutput(March25_P.DigitalOutput_pinNumber_c);
}

/* Model terminate function */
void March25_terminate(void)
{
  /* (no terminate code required) */
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
