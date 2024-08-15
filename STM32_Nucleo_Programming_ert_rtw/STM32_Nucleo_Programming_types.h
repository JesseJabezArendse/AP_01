/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: STM32_Nucleo_Programming_types.h
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

#ifndef RTW_HEADER_STM32_Nucleo_Programming_types_h_
#define RTW_HEADER_STM32_Nucleo_Programming_types_h_
#include "rtwtypes.h"
#include "MW_SVD.h"

/* Custom Type definition for MATLABSystem: '<S5>/Magnetometer initialization' */
#include "MW_SVD.h"
#ifndef struct_tag_5FwKk6wA1XPbMoI1XCDeDF
#define struct_tag_5FwKk6wA1XPbMoI1XCDeDF

struct tag_5FwKk6wA1XPbMoI1XCDeDF
{
  boolean_T matlabCodegenIsDeleted;
  int32_T isInitialized;
  boolean_T isSetupComplete;
  MW_Handle_Type MW_DIGITALIO_HANDLE;
  real_T SampleTime;
};

#endif                                 /* struct_tag_5FwKk6wA1XPbMoI1XCDeDF */

#ifndef typedef_mbed_DigitalRead_STM32_Nucleo_T
#define typedef_mbed_DigitalRead_STM32_Nucleo_T

typedef struct tag_5FwKk6wA1XPbMoI1XCDeDF mbed_DigitalRead_STM32_Nucleo_T;

#endif                             /* typedef_mbed_DigitalRead_STM32_Nucleo_T */

#ifndef struct_tag_KxFW01GBdhqk5JOEHU3GlD
#define struct_tag_KxFW01GBdhqk5JOEHU3GlD

struct tag_KxFW01GBdhqk5JOEHU3GlD
{
  boolean_T matlabCodegenIsDeleted;
  int32_T isInitialized;
  boolean_T isSetupComplete;
  MW_Handle_Type MW_DIGITALIO_HANDLE;
};

#endif                                 /* struct_tag_KxFW01GBdhqk5JOEHU3GlD */

#ifndef typedef_mbed_DigitalWrite_STM32_Nucle_T
#define typedef_mbed_DigitalWrite_STM32_Nucle_T

typedef struct tag_KxFW01GBdhqk5JOEHU3GlD mbed_DigitalWrite_STM32_Nucle_T;

#endif                             /* typedef_mbed_DigitalWrite_STM32_Nucle_T */

#ifndef struct_tag_45NO01sr76bOLMYCTk1zwH
#define struct_tag_45NO01sr76bOLMYCTk1zwH

struct tag_45NO01sr76bOLMYCTk1zwH
{
  real_T currentTime;
};

#endif                                 /* struct_tag_45NO01sr76bOLMYCTk1zwH */

#ifndef typedef_f_codertarget_mbed_internal_M_T
#define typedef_f_codertarget_mbed_internal_M_T

typedef struct tag_45NO01sr76bOLMYCTk1zwH f_codertarget_mbed_internal_M_T;

#endif                             /* typedef_f_codertarget_mbed_internal_M_T */

#ifndef struct_tag_aKWq6a6RxmYGQum8qfKjlF
#define struct_tag_aKWq6a6RxmYGQum8qfKjlF

struct tag_aKWq6a6RxmYGQum8qfKjlF
{
  MW_Handle_Type MW_I2C_HANDLE;
};

#endif                                 /* struct_tag_aKWq6a6RxmYGQum8qfKjlF */

#ifndef typedef_f_matlabshared_devicedrivers__T
#define typedef_f_matlabshared_devicedrivers__T

typedef struct tag_aKWq6a6RxmYGQum8qfKjlF f_matlabshared_devicedrivers__T;

#endif                             /* typedef_f_matlabshared_devicedrivers__T */

#ifndef struct_tag_MYJ770bjSLi75gskT6UTJF
#define struct_tag_MYJ770bjSLi75gskT6UTJF

struct tag_MYJ770bjSLi75gskT6UTJF
{
  uint8_T Bus;
  uint8_T DeviceAddress;
  f_matlabshared_devicedrivers__T *InterfaceObj;
  f_matlabshared_devicedrivers__T _pobj0;
};

#endif                                 /* struct_tag_MYJ770bjSLi75gskT6UTJF */

#ifndef typedef_f_matlabshared_sensors_coder__T
#define typedef_f_matlabshared_sensors_coder__T

typedef struct tag_MYJ770bjSLi75gskT6UTJF f_matlabshared_sensors_coder__T;

#endif                             /* typedef_f_matlabshared_sensors_coder__T */

#ifndef struct_tag_WO6UnBrSRuV3bp2XHEOxRF
#define struct_tag_WO6UnBrSRuV3bp2XHEOxRF

struct tag_WO6UnBrSRuV3bp2XHEOxRF
{
  boolean_T matlabCodegenIsDeleted;
  int32_T isInitialized;
  f_codertarget_mbed_internal_M_T *Parent;
  f_matlabshared_sensors_coder__T *Device;
  real_T AccelerometerResolution;
  real_T GyroscopeResolution;
  f_matlabshared_sensors_coder__T _pobj0;
};

#endif                                 /* struct_tag_WO6UnBrSRuV3bp2XHEOxRF */

#ifndef typedef_b_lsm6dsl_STM32_Nucleo_Progra_T
#define typedef_b_lsm6dsl_STM32_Nucleo_Progra_T

typedef struct tag_WO6UnBrSRuV3bp2XHEOxRF b_lsm6dsl_STM32_Nucleo_Progra_T;

#endif                             /* typedef_b_lsm6dsl_STM32_Nucleo_Progra_T */

#ifndef struct_tag_As8gYj3mjzMic4nXDF1rQG
#define struct_tag_As8gYj3mjzMic4nXDF1rQG

struct tag_As8gYj3mjzMic4nXDF1rQG
{
  int32_T __dummy;
};

#endif                                 /* struct_tag_As8gYj3mjzMic4nXDF1rQG */

#ifndef typedef_l_matlabshared_sensors_simuli_T
#define typedef_l_matlabshared_sensors_simuli_T

typedef struct tag_As8gYj3mjzMic4nXDF1rQG l_matlabshared_sensors_simuli_T;

#endif                             /* typedef_l_matlabshared_sensors_simuli_T */

#ifndef struct_tag_AUqvznWQtxWcnYF6XmyZp
#define struct_tag_AUqvznWQtxWcnYF6XmyZp

struct tag_AUqvznWQtxWcnYF6XmyZp
{
  int32_T __dummy;
};

#endif                                 /* struct_tag_AUqvznWQtxWcnYF6XmyZp */

#ifndef typedef_m_matlabshared_sensors_simuli_T
#define typedef_m_matlabshared_sensors_simuli_T

typedef struct tag_AUqvznWQtxWcnYF6XmyZp m_matlabshared_sensors_simuli_T;

#endif                             /* typedef_m_matlabshared_sensors_simuli_T */

#ifndef struct_tag_emaM8BrQJAuATZkkqWHyXF
#define struct_tag_emaM8BrQJAuATZkkqWHyXF

struct tag_emaM8BrQJAuATZkkqWHyXF
{
  int32_T __dummy;
};

#endif                                 /* struct_tag_emaM8BrQJAuATZkkqWHyXF */

#ifndef typedef_n_matlabshared_sensors_simuli_T
#define typedef_n_matlabshared_sensors_simuli_T

typedef struct tag_emaM8BrQJAuATZkkqWHyXF n_matlabshared_sensors_simuli_T;

#endif                             /* typedef_n_matlabshared_sensors_simuli_T */

#ifndef struct_tag_L7Rl3yAuleXKUd99BBfg8B
#define struct_tag_L7Rl3yAuleXKUd99BBfg8B

struct tag_L7Rl3yAuleXKUd99BBfg8B
{
  int32_T __dummy;
};

#endif                                 /* struct_tag_L7Rl3yAuleXKUd99BBfg8B */

#ifndef typedef_o_matlabshared_sensors_simuli_T
#define typedef_o_matlabshared_sensors_simuli_T

typedef struct tag_L7Rl3yAuleXKUd99BBfg8B o_matlabshared_sensors_simuli_T;

#endif                             /* typedef_o_matlabshared_sensors_simuli_T */

#ifndef struct_tag_8UbgSWsThsm6R57RiBI8XD
#define struct_tag_8UbgSWsThsm6R57RiBI8XD

struct tag_8UbgSWsThsm6R57RiBI8XD
{
  l_matlabshared_sensors_simuli_T *f1;
  m_matlabshared_sensors_simuli_T *f2;
  n_matlabshared_sensors_simuli_T *f3;
  o_matlabshared_sensors_simuli_T *f4;
};

#endif                                 /* struct_tag_8UbgSWsThsm6R57RiBI8XD */

#ifndef typedef_fb_cell_STM32_Nucleo_Programm_T
#define typedef_fb_cell_STM32_Nucleo_Programm_T

typedef struct tag_8UbgSWsThsm6R57RiBI8XD fb_cell_STM32_Nucleo_Programm_T;

#endif                             /* typedef_fb_cell_STM32_Nucleo_Programm_T */

#ifndef struct_tag_YLMe8IJ14eMynLprVmsMUD
#define struct_tag_YLMe8IJ14eMynLprVmsMUD

struct tag_YLMe8IJ14eMynLprVmsMUD
{
  boolean_T matlabCodegenIsDeleted;
  int32_T isInitialized;
  boolean_T isSetupComplete;
  boolean_T TunablePropsChanged;
  real_T SampleTime;
  f_codertarget_mbed_internal_M_T *HwUtilityObject;
  b_lsm6dsl_STM32_Nucleo_Progra_T *SensorObject;
  fb_cell_STM32_Nucleo_Programm_T OutputModules;
  boolean_T __OutputModules_AssignmentSentinel;
  o_matlabshared_sensors_simuli_T _pobj0;
  n_matlabshared_sensors_simuli_T _pobj1;
  m_matlabshared_sensors_simuli_T _pobj2;
  l_matlabshared_sensors_simuli_T _pobj3;
  b_lsm6dsl_STM32_Nucleo_Progra_T _pobj4;
  f_codertarget_mbed_internal_M_T _pobj5;
};

#endif                                 /* struct_tag_YLMe8IJ14eMynLprVmsMUD */

#ifndef typedef_mbed_LSM6DSLBlock_STM32_Nucle_T
#define typedef_mbed_LSM6DSLBlock_STM32_Nucle_T

typedef struct tag_YLMe8IJ14eMynLprVmsMUD mbed_LSM6DSLBlock_STM32_Nucle_T;

#endif                             /* typedef_mbed_LSM6DSLBlock_STM32_Nucle_T */

/* Custom Type definition for MATLABSystem: '<S5>/Magnetometer initialization' */
#include "MW_I2C.h"
#ifndef struct_tag_H1xN04M2SFjxqRFFmCjSXC
#define struct_tag_H1xN04M2SFjxqRFFmCjSXC

struct tag_H1xN04M2SFjxqRFFmCjSXC
{
  boolean_T matlabCodegenIsDeleted;
  int32_T isInitialized;
  boolean_T isSetupComplete;
  uint32_T BusSpeed;
  real_T DefaultMaximumBusSpeedInHz;
  MW_Handle_Type MW_I2C_HANDLE;
  real_T SampleTime;
};

#endif                                 /* struct_tag_H1xN04M2SFjxqRFFmCjSXC */

#ifndef typedef_mbed_I2CMasterRead_STM32_Nucl_T
#define typedef_mbed_I2CMasterRead_STM32_Nucl_T

typedef struct tag_H1xN04M2SFjxqRFFmCjSXC mbed_I2CMasterRead_STM32_Nucl_T;

#endif                             /* typedef_mbed_I2CMasterRead_STM32_Nucl_T */

#ifndef struct_tag_YSdb5T318gvwi5OzjiD6SE
#define struct_tag_YSdb5T318gvwi5OzjiD6SE

struct tag_YSdb5T318gvwi5OzjiD6SE
{
  boolean_T matlabCodegenIsDeleted;
  int32_T isInitialized;
  boolean_T isSetupComplete;
  uint32_T BusSpeed;
  real_T DefaultMaximumBusSpeedInHz;
  MW_Handle_Type MW_I2C_HANDLE;
};

#endif                                 /* struct_tag_YSdb5T318gvwi5OzjiD6SE */

#ifndef typedef_mbed_I2CMasterWrite_STM32_Nuc_T
#define typedef_mbed_I2CMasterWrite_STM32_Nuc_T

typedef struct tag_YSdb5T318gvwi5OzjiD6SE mbed_I2CMasterWrite_STM32_Nuc_T;

#endif                             /* typedef_mbed_I2CMasterWrite_STM32_Nuc_T */

/* Parameters (default storage) */
typedef struct P_STM32_Nucleo_Programming_T_ P_STM32_Nucleo_Programming_T;

/* Forward declaration for rtModel */
typedef struct tag_RTM_STM32_Nucleo_Programm_T RT_MODEL_STM32_Nucleo_Program_T;

#endif                        /* RTW_HEADER_STM32_Nucleo_Programming_types_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
