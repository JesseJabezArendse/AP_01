/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: STM32_Nucleo_Programming.h
 *
 * Code generated for Simulink model 'STM32_Nucleo_Programming'.
 *
 * Model version                  : 1.26
 * Simulink Coder version         : 9.9 (R2023a) 19-Nov-2022
 * C/C++ source code generated on : Wed Aug 14 08:27:27 2024
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: ARM Compatible->ARM Cortex
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#ifndef RTW_HEADER_STM32_Nucleo_Programming_h_
#define RTW_HEADER_STM32_Nucleo_Programming_h_
#ifndef STM32_Nucleo_Programming_COMMON_INCLUDES_
#define STM32_Nucleo_Programming_COMMON_INCLUDES_
#include "rtwtypes.h"
#include "rtw_continuous.h"
#include "rtw_solver.h"
#include "MW_MbedPinInterface.h"
#include "MW_digitalIO.h"
#include "MW_I2C.h"
#endif                           /* STM32_Nucleo_Programming_COMMON_INCLUDES_ */

#include "STM32_Nucleo_Programming_types.h"
#include <stddef.h>
#include "MW_target_hardware_resources.h"

/* Macros for accessing real-time model data structure */
#ifndef rtmGetErrorStatus
#define rtmGetErrorStatus(rtm)         ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
#define rtmSetErrorStatus(rtm, val)    ((rtm)->errorStatus = (val))
#endif

/* user code (top of header file) */
#include "NucleoMX\Core\Inc\CustomDump.h"
#include "NucleoMX\Core\Inc\usart.h"
#include "NucleoMX\Core\Inc\main.h"

/* Block signals (default storage) */
typedef struct {
  MW_I2C_Mode_Type ModeType;
  uint32_T i2cname;
  MW_I2C_Mode_Type modename;
  int32_T i;
  uint32_T intMask;
} B_STM32_Nucleo_Programming_T;

/* Block states (default storage) for system '<Root>' */
typedef struct {
  mbed_LSM6DSLBlock_STM32_Nucle_T obj; /* '<S2>/LSM6DSL IMU Sensor' */
  mbed_I2CMasterRead_STM32_Nucl_T obj_p;/* '<S3>/I2C Controller Read' */
  mbed_DigitalRead_STM32_Nucleo_T obj_c;/* '<Root>/Blue Button' */
  mbed_I2CMasterWrite_STM32_Nuc_T obj_m;/* '<S5>/Magnetometer initialization' */
  mbed_DigitalWrite_STM32_Nucle_T obj_k;/* '<Root>/Green LED' */
} DW_STM32_Nucleo_Programming_T;

/* Parameters (default storage) */
struct P_STM32_Nucleo_Programming_T_ {
  real_T BlueButton_SampleTime;        /* Expression: -1
                                        * Referenced by: '<Root>/Blue Button'
                                        */
  real_T LSM6DSLIMUSensor_SampleTime;  /* Expression: 0.01
                                        * Referenced by: '<S2>/LSM6DSL IMU Sensor'
                                        */
  real_T I2CControllerRead_SampleTime; /* Expression: 0.01
                                        * Referenced by: '<S3>/I2C Controller Read'
                                        */
  real_T DataStoreMemory_InitialValue; /* Expression: 0
                                        * Referenced by: '<S2>/Data Store Memory'
                                        */
  real_T DataStoreMemory1_InitialValue;/* Expression: 0
                                        * Referenced by: '<S2>/Data Store Memory1'
                                        */
  real_T DataStoreMemory2_InitialValue;/* Expression: 0
                                        * Referenced by: '<S2>/Data Store Memory2'
                                        */
  int16_T DataStoreMemory_InitialValue_k;
                           /* Computed Parameter: DataStoreMemory_InitialValue_k
                            * Referenced by: '<S3>/Data Store Memory'
                            */
  uint16_T DataStoreMemory_InitialValue_g;
                           /* Computed Parameter: DataStoreMemory_InitialValue_g
                            * Referenced by: '<S4>/Data Store Memory'
                            */
  uint8_T Constant_Value;              /* Expression: 0x8C
                                        * Referenced by: '<S5>/Constant'
                                        */
};

/* Real-time Model Data Structure */
struct tag_RTM_STM32_Nucleo_Programm_T {
  const char_T * volatile errorStatus;
};

/* Block parameters (default storage) */
extern P_STM32_Nucleo_Programming_T STM32_Nucleo_Programming_P;

/* Block signals (default storage) */
extern B_STM32_Nucleo_Programming_T STM32_Nucleo_Programming_B;

/* Block states (default storage) */
extern DW_STM32_Nucleo_Programming_T STM32_Nucleo_Programming_DW;

/*
 * Exported States
 *
 * Note: Exported states are block states with an exported global
 * storage class designation.  Code generation will declare the memory for these
 * states and exports their symbols.
 *
 */
extern real_T IMU_Accel[3];            /* '<S2>/Data Store Memory' */
extern real_T IMU_Gyro[3];             /* '<S2>/Data Store Memory1' */
extern real_T IMU_Temp;                /* '<S2>/Data Store Memory2' */
extern int16_T Magnetometer[3];        /* '<S3>/Data Store Memory' */
extern uint16_T ToF[64];               /* '<S4>/Data Store Memory' */

/* Model entry point functions */
extern void STM32_Nucleo_Programming_initialize(void);
extern void STM32_Nucleo_Programming_step(void);
extern void STM32_Nucleo_Programming_terminate(void);

/* Real-time Model object */
extern RT_MODEL_STM32_Nucleo_Program_T *const STM32_Nucleo_Programming_M;
extern volatile boolean_T stopRequested;
extern volatile boolean_T runModel;

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
 * '<Root>' : 'STM32_Nucleo_Programming'
 * '<S1>'   : 'STM32_Nucleo_Programming/Data Dump'
 * '<S2>'   : 'STM32_Nucleo_Programming/IMU'
 * '<S3>'   : 'STM32_Nucleo_Programming/Magnetometer'
 * '<S4>'   : 'STM32_Nucleo_Programming/Time of Flight'
 * '<S5>'   : 'STM32_Nucleo_Programming/Magnetometer/Magnetometer initialization'
 */
#endif                              /* RTW_HEADER_STM32_Nucleo_Programming_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
