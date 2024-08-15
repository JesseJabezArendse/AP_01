/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: STM32_Nucleo_Programming.c
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

#include "STM32_Nucleo_Programming.h"
#include "STM32_Nucleo_Programming_types.h"
#include <string.h>
#include "rtwtypes.h"
#include <stddef.h>

/* user code (top of source file) */
/* System '<Root>' */
UART_HandleTypeDef huart2;

/* Exported block states */
real_T IMU_Accel[3];                   /* '<S2>/Data Store Memory' */
real_T IMU_Gyro[3];                    /* '<S2>/Data Store Memory1' */
real_T IMU_Temp;                       /* '<S2>/Data Store Memory2' */
int16_T Magnetometer[3];               /* '<S3>/Data Store Memory' */
uint16_T ToF[64];                      /* '<S4>/Data Store Memory' */

/* Block signals (default storage) */
B_STM32_Nucleo_Programming_T STM32_Nucleo_Programming_B;

/* Block states (default storage) */
DW_STM32_Nucleo_Programming_T STM32_Nucleo_Programming_DW;

/* Real-time model */
static RT_MODEL_STM32_Nucleo_Program_T STM32_Nucleo_Programming_M_;
RT_MODEL_STM32_Nucleo_Program_T *const STM32_Nucleo_Programming_M =
  &STM32_Nucleo_Programming_M_;

/* Forward declaration for local functions */
static void STM32_Nucleo_P_SystemCore_setup(mbed_LSM6DSLBlock_STM32_Nucle_T *obj);
static void STM32_Nucleo_P_SystemCore_setup(mbed_LSM6DSLBlock_STM32_Nucle_T *obj)
{
  f_codertarget_mbed_internal_M_T *varargin_1;
  f_matlabshared_devicedrivers__T *obj_1;
  f_matlabshared_sensors_coder__T *obj_0;
  uint8_T b_data[2];
  uint8_T b_status;
  uint8_T i2cModule;
  uint8_T val;
  obj->isSetupComplete = false;
  obj->isInitialized = 1;
  STM32_Nucleo_Programming_B.intMask = __get_PRIMASK();
  __enable_irq();
  obj->_pobj5.currentTime = -1.0;
  obj->HwUtilityObject = &obj->_pobj5;
  varargin_1 = obj->HwUtilityObject;
  obj->_pobj4.isInitialized = 0;
  obj->_pobj4.Parent = varargin_1;
  obj->_pobj4._pobj0.Bus = 0U;
  obj->_pobj4._pobj0.DeviceAddress = 107U;
  obj->_pobj4._pobj0._pobj0.MW_I2C_HANDLE = NULL;
  obj->_pobj4._pobj0.InterfaceObj = &obj->_pobj4._pobj0._pobj0;
  obj_1 = obj->_pobj4._pobj0.InterfaceObj;
  i2cModule = obj->_pobj4._pobj0.Bus;
  STM32_Nucleo_Programming_B.modename = MW_I2C_MASTER;
  obj_1->MW_I2C_HANDLE = MW_I2C_Open(i2cModule,
    STM32_Nucleo_Programming_B.modename);
  obj->_pobj4.Device = &obj->_pobj4._pobj0;
  obj_0 = obj->_pobj4.Device;
  obj_1 = obj_0->InterfaceObj;
  i2cModule = obj_0->DeviceAddress;
  b_status = 15U;
  b_status = MW_I2C_MasterWrite(obj_1->MW_I2C_HANDLE, i2cModule, &b_status, 1U,
    true, false);
  if (b_status == 0) {
    MW_I2C_MasterRead(obj_1->MW_I2C_HANDLE, i2cModule, &b_status, 1U, false,
                      true);
  }

  obj_0 = obj->_pobj4.Device;
  obj_1 = obj_0->InterfaceObj;
  i2cModule = obj_0->DeviceAddress;
  b_data[0] = 24U;
  b_data[1] = 56U;
  MW_I2C_MasterWrite(obj_1->MW_I2C_HANDLE, i2cModule, &b_data[0], 2U, false,
                     false);
  obj_0 = obj->_pobj4.Device;
  obj_1 = obj_0->InterfaceObj;
  i2cModule = obj_0->DeviceAddress;
  b_data[0] = 25U;
  b_data[1] = 56U;
  MW_I2C_MasterWrite(obj_1->MW_I2C_HANDLE, i2cModule, &b_data[0], 2U, false,
                     false);
  obj->_pobj4.AccelerometerResolution = 0.0001220703125;
  obj_0 = obj->_pobj4.Device;
  b_status = 16U;
  obj_1 = obj_0->InterfaceObj;
  i2cModule = obj_0->DeviceAddress;
  val = 0U;
  b_status = MW_I2C_MasterWrite(obj_1->MW_I2C_HANDLE, i2cModule, &b_status, 1U,
    true, false);
  if (b_status == 0) {
    MW_I2C_MasterRead(obj_1->MW_I2C_HANDLE, i2cModule, &val, 1U, false, true);
  }

  obj_0 = obj->_pobj4.Device;
  obj_1 = obj_0->InterfaceObj;
  i2cModule = obj_0->DeviceAddress;
  b_data[0] = 16U;
  b_data[1] = (uint8_T)((val & 243) | 8);
  MW_I2C_MasterWrite(obj_1->MW_I2C_HANDLE, i2cModule, &b_data[0], 2U, false,
                     false);
  obj_0 = obj->_pobj4.Device;
  b_status = 16U;
  obj_1 = obj_0->InterfaceObj;
  i2cModule = obj_0->DeviceAddress;
  val = 0U;
  b_status = MW_I2C_MasterWrite(obj_1->MW_I2C_HANDLE, i2cModule, &b_status, 1U,
    true, false);
  if (b_status == 0) {
    MW_I2C_MasterRead(obj_1->MW_I2C_HANDLE, i2cModule, &val, 1U, false, true);
  }

  obj_0 = obj->_pobj4.Device;
  obj_1 = obj_0->InterfaceObj;
  i2cModule = obj_0->DeviceAddress;
  b_data[0] = 16U;
  b_data[1] = (uint8_T)((val & 15) | 64);
  MW_I2C_MasterWrite(obj_1->MW_I2C_HANDLE, i2cModule, &b_data[0], 2U, false,
                     false);
  obj_0 = obj->_pobj4.Device;
  b_status = 16U;
  obj_1 = obj_0->InterfaceObj;
  i2cModule = obj_0->DeviceAddress;
  val = 0U;
  b_status = MW_I2C_MasterWrite(obj_1->MW_I2C_HANDLE, i2cModule, &b_status, 1U,
    true, false);
  if (b_status == 0) {
    MW_I2C_MasterRead(obj_1->MW_I2C_HANDLE, i2cModule, &val, 1U, false, true);
  }

  obj_0 = obj->_pobj4.Device;
  obj_1 = obj_0->InterfaceObj;
  i2cModule = obj_0->DeviceAddress;
  b_data[0] = 16U;
  b_data[1] = (uint8_T)(val & 253);
  MW_I2C_MasterWrite(obj_1->MW_I2C_HANDLE, i2cModule, &b_data[0], 2U, false,
                     false);
  obj_0 = obj->_pobj4.Device;
  b_status = 23U;
  obj_1 = obj_0->InterfaceObj;
  i2cModule = obj_0->DeviceAddress;
  val = 0U;
  b_status = MW_I2C_MasterWrite(obj_1->MW_I2C_HANDLE, i2cModule, &b_status, 1U,
    true, false);
  if (b_status == 0) {
    MW_I2C_MasterRead(obj_1->MW_I2C_HANDLE, i2cModule, &val, 1U, false, true);
  }

  obj_0 = obj->_pobj4.Device;
  obj_1 = obj_0->InterfaceObj;
  i2cModule = obj_0->DeviceAddress;
  b_data[0] = 23U;
  b_data[1] = (uint8_T)(val & 247);
  MW_I2C_MasterWrite(obj_1->MW_I2C_HANDLE, i2cModule, &b_data[0], 2U, false,
                     false);
  obj->_pobj4.GyroscopeResolution = 0.00875;
  obj_0 = obj->_pobj4.Device;
  b_status = 17U;
  obj_1 = obj_0->InterfaceObj;
  i2cModule = obj_0->DeviceAddress;
  val = 0U;
  b_status = MW_I2C_MasterWrite(obj_1->MW_I2C_HANDLE, i2cModule, &b_status, 1U,
    true, false);
  if (b_status == 0) {
    MW_I2C_MasterRead(obj_1->MW_I2C_HANDLE, i2cModule, &val, 1U, false, true);
  }

  obj_0 = obj->_pobj4.Device;
  obj_1 = obj_0->InterfaceObj;
  i2cModule = obj_0->DeviceAddress;
  b_data[0] = 17U;
  b_data[1] = (uint8_T)(val & 240);
  MW_I2C_MasterWrite(obj_1->MW_I2C_HANDLE, i2cModule, &b_data[0], 2U, false,
                     false);
  obj_0 = obj->_pobj4.Device;
  b_status = 17U;
  obj_1 = obj_0->InterfaceObj;
  i2cModule = obj_0->DeviceAddress;
  val = 0U;
  b_status = MW_I2C_MasterWrite(obj_1->MW_I2C_HANDLE, i2cModule, &b_status, 1U,
    true, false);
  if (b_status == 0) {
    MW_I2C_MasterRead(obj_1->MW_I2C_HANDLE, i2cModule, &val, 1U, false, true);
  }

  obj_0 = obj->_pobj4.Device;
  obj_1 = obj_0->InterfaceObj;
  i2cModule = obj_0->DeviceAddress;
  b_data[0] = 17U;
  b_data[1] = (uint8_T)((val & 15) | 64);
  MW_I2C_MasterWrite(obj_1->MW_I2C_HANDLE, i2cModule, &b_data[0], 2U, false,
                     false);
  obj_0 = obj->_pobj4.Device;
  b_status = 19U;
  obj_1 = obj_0->InterfaceObj;
  i2cModule = obj_0->DeviceAddress;
  val = 0U;
  b_status = MW_I2C_MasterWrite(obj_1->MW_I2C_HANDLE, i2cModule, &b_status, 1U,
    true, false);
  if (b_status == 0) {
    MW_I2C_MasterRead(obj_1->MW_I2C_HANDLE, i2cModule, &val, 1U, false, true);
  }

  obj_0 = obj->_pobj4.Device;
  obj_1 = obj_0->InterfaceObj;
  i2cModule = obj_0->DeviceAddress;
  b_data[0] = 19U;
  b_data[1] = (uint8_T)(val & 253);
  MW_I2C_MasterWrite(obj_1->MW_I2C_HANDLE, i2cModule, &b_data[0], 2U, false,
                     false);
  obj->_pobj4.matlabCodegenIsDeleted = false;
  obj->SensorObject = &obj->_pobj4;
  obj->OutputModules.f1 = &obj->_pobj3;
  obj->OutputModules.f2 = &obj->_pobj2;
  obj->OutputModules.f3 = &obj->_pobj1;
  obj->OutputModules.f4 = &obj->_pobj0;
  __set_PRIMASK(STM32_Nucleo_Programming_B.intMask);
  obj->isSetupComplete = true;
  obj->TunablePropsChanged = false;
}

/* Model step function */
void STM32_Nucleo_Programming_step(void)
{
  b_lsm6dsl_STM32_Nucleo_Progra_T *sensorObj_tmp;
  f_matlabshared_devicedrivers__T *obj_0;
  f_matlabshared_sensors_coder__T *obj;
  int32_T i;
  uint8_T output_raw[6];
  uint8_T tempData[2];
  uint8_T b_status;
  uint8_T status;
  boolean_T rtb_BlueButton_0;

  /* MATLABSystem: '<S2>/LSM6DSL IMU Sensor' incorporates:
   *  DataStoreWrite: '<S2>/Data Store Write2'
   */
  if (STM32_Nucleo_Programming_DW.obj.SampleTime !=
      STM32_Nucleo_Programming_P.LSM6DSLIMUSensor_SampleTime) {
    STM32_Nucleo_Programming_DW.obj.SampleTime =
      STM32_Nucleo_Programming_P.LSM6DSLIMUSensor_SampleTime;
  }

  if (STM32_Nucleo_Programming_DW.obj.TunablePropsChanged) {
    STM32_Nucleo_Programming_DW.obj.TunablePropsChanged = false;
  }

  sensorObj_tmp = STM32_Nucleo_Programming_DW.obj.SensorObject;
  obj = sensorObj_tmp->Device;
  obj_0 = obj->InterfaceObj;
  status = obj->DeviceAddress;
  b_status = 30U;
  b_status = MW_I2C_MasterWrite(obj_0->MW_I2C_HANDLE, status, &b_status, 1U,
    true, false);
  if (b_status == 0) {
    MW_I2C_MasterRead(obj_0->MW_I2C_HANDLE, status, &b_status, 1U, false, true);
  }

  obj = sensorObj_tmp->Device;
  obj_0 = obj->InterfaceObj;
  status = obj->DeviceAddress;
  for (i = 0; i < 6; i++) {
    output_raw[i] = 0U;
  }

  b_status = 40U;
  b_status = MW_I2C_MasterWrite(obj_0->MW_I2C_HANDLE, status, &b_status, 1U,
    true, false);
  if (b_status == 0) {
    MW_I2C_MasterRead(obj_0->MW_I2C_HANDLE, status, &output_raw[0], 6U, false,
                      true);
  }

  IMU_Accel[0] = (real_T)((int16_T)(output_raw[1] << 8) | output_raw[0]) *
    sensorObj_tmp->AccelerometerResolution;
  IMU_Accel[1] = (real_T)((int16_T)(output_raw[3] << 8) | output_raw[2]) *
    sensorObj_tmp->AccelerometerResolution;
  IMU_Accel[2] = (real_T)((int16_T)(output_raw[5] << 8) | output_raw[4]) *
    sensorObj_tmp->AccelerometerResolution;
  obj = sensorObj_tmp->Device;
  obj_0 = obj->InterfaceObj;
  status = obj->DeviceAddress;
  for (i = 0; i < 6; i++) {
    output_raw[i] = 0U;
  }

  b_status = 34U;
  b_status = MW_I2C_MasterWrite(obj_0->MW_I2C_HANDLE, status, &b_status, 1U,
    true, false);
  if (b_status == 0) {
    MW_I2C_MasterRead(obj_0->MW_I2C_HANDLE, status, &output_raw[0], 6U, false,
                      true);
  }

  IMU_Gyro[0] = (real_T)((int16_T)(output_raw[1] << 8) | output_raw[0]) *
    sensorObj_tmp->GyroscopeResolution;
  IMU_Gyro[1] = (real_T)((int16_T)(output_raw[3] << 8) | output_raw[2]) *
    sensorObj_tmp->GyroscopeResolution;
  IMU_Gyro[2] = (real_T)((int16_T)(output_raw[5] << 8) | output_raw[4]) *
    sensorObj_tmp->GyroscopeResolution;
  IMU_Gyro[0] = IMU_Gyro[0] * 3.1415926535897931 / 180.0;
  IMU_Gyro[1] = IMU_Gyro[1] * 3.1415926535897931 / 180.0;
  IMU_Gyro[2] = IMU_Gyro[2] * 3.1415926535897931 / 180.0;
  obj = sensorObj_tmp->Device;
  obj_0 = obj->InterfaceObj;
  status = obj->DeviceAddress;
  tempData[0] = 0U;
  tempData[1] = 0U;
  b_status = 32U;
  b_status = MW_I2C_MasterWrite(obj_0->MW_I2C_HANDLE, status, &b_status, 1U,
    true, false);
  if (b_status == 0) {
    MW_I2C_MasterRead(obj_0->MW_I2C_HANDLE, status, &tempData[0], 2U, false,
                      true);
  }

  IMU_Temp = (real_T)((int16_T)(tempData[1] << 8) | tempData[0]) * 0.00390625 +
    25.0;

  /* DataStoreWrite: '<S2>/Data Store Write' incorporates:
   *  MATLABSystem: '<S2>/LSM6DSL IMU Sensor'
   */
  IMU_Accel[0] *= 9.81;
  IMU_Accel[1] *= 9.81;
  IMU_Accel[2] *= 9.81;

  /* MATLABSystem: '<S3>/I2C Controller Read' incorporates:
   *  DataStoreWrite: '<S3>/Data Store Write'
   */
  if (STM32_Nucleo_Programming_DW.obj_p.SampleTime !=
      STM32_Nucleo_Programming_P.I2CControllerRead_SampleTime) {
    STM32_Nucleo_Programming_DW.obj_p.SampleTime =
      STM32_Nucleo_Programming_P.I2CControllerRead_SampleTime;
  }

  status = 104U;
  memcpy((void *)&b_status, (void *)&status, (size_t)1 * sizeof(uint8_T));
  status = MW_I2C_MasterWrite(STM32_Nucleo_Programming_DW.obj_p.MW_I2C_HANDLE,
    30U, &b_status, 1U, true, false);
  if (status == 0) {
    MW_I2C_MasterRead(STM32_Nucleo_Programming_DW.obj_p.MW_I2C_HANDLE, 30U,
                      &output_raw[0], 6U, false, true);
    memcpy((void *)&Magnetometer[0], (void *)&output_raw[0], (size_t)3 * sizeof
           (int16_T));
  } else {
    /* DataStoreWrite: '<S3>/Data Store Write' */
    Magnetometer[0] = 0;
    Magnetometer[1] = 0;
    Magnetometer[2] = 0;
  }

  /* End of MATLABSystem: '<S3>/I2C Controller Read' */

  /* DataStoreWrite: '<S4>/Data Store Write' */
  memset(&ToF[0], 0, sizeof(uint16_T) << 6U);

  /* MATLABSystem: '<Root>/Blue Button' */
  if (STM32_Nucleo_Programming_DW.obj_c.SampleTime !=
      STM32_Nucleo_Programming_P.BlueButton_SampleTime) {
    STM32_Nucleo_Programming_DW.obj_c.SampleTime =
      STM32_Nucleo_Programming_P.BlueButton_SampleTime;
  }

  rtb_BlueButton_0 = MW_digitalIO_read
    (STM32_Nucleo_Programming_DW.obj_c.MW_DIGITALIO_HANDLE);

  /* MATLABSystem: '<Root>/Green LED' incorporates:
   *  MATLABSystem: '<Root>/Blue Button'
   */
  MW_digitalIO_write(STM32_Nucleo_Programming_DW.obj_k.MW_DIGITALIO_HANDLE,
                     rtb_BlueButton_0);

  /* user code (Update function Body for TID0) */

  /* System '<Root>' */
  Dump_IKS02A1();
}

/* Model initialize function */
void STM32_Nucleo_Programming_initialize(void)
{
  {
    mbed_DigitalRead_STM32_Nucleo_T *obj_0;
    mbed_DigitalWrite_STM32_Nucle_T *obj_1;
    mbed_I2CMasterRead_STM32_Nucl_T *obj;
    mbed_I2CMasterWrite_STM32_Nuc_T *obj_2;
    uint8_T b_SwappedDataBytes[2];

    /* Start for DataStoreMemory: '<S2>/Data Store Memory2' */
    IMU_Temp = STM32_Nucleo_Programming_P.DataStoreMemory2_InitialValue;

    /* Start for DataStoreMemory: '<S2>/Data Store Memory' */
    IMU_Accel[0] = STM32_Nucleo_Programming_P.DataStoreMemory_InitialValue;

    /* Start for DataStoreMemory: '<S2>/Data Store Memory1' */
    IMU_Gyro[0] = STM32_Nucleo_Programming_P.DataStoreMemory1_InitialValue;

    /* Start for DataStoreMemory: '<S3>/Data Store Memory' */
    Magnetometer[0] = STM32_Nucleo_Programming_P.DataStoreMemory_InitialValue_k;

    /* Start for DataStoreMemory: '<S2>/Data Store Memory' */
    IMU_Accel[1] = STM32_Nucleo_Programming_P.DataStoreMemory_InitialValue;

    /* Start for DataStoreMemory: '<S2>/Data Store Memory1' */
    IMU_Gyro[1] = STM32_Nucleo_Programming_P.DataStoreMemory1_InitialValue;

    /* Start for DataStoreMemory: '<S3>/Data Store Memory' */
    Magnetometer[1] = STM32_Nucleo_Programming_P.DataStoreMemory_InitialValue_k;

    /* Start for DataStoreMemory: '<S2>/Data Store Memory' */
    IMU_Accel[2] = STM32_Nucleo_Programming_P.DataStoreMemory_InitialValue;

    /* Start for DataStoreMemory: '<S2>/Data Store Memory1' */
    IMU_Gyro[2] = STM32_Nucleo_Programming_P.DataStoreMemory1_InitialValue;

    /* Start for DataStoreMemory: '<S3>/Data Store Memory' */
    Magnetometer[2] = STM32_Nucleo_Programming_P.DataStoreMemory_InitialValue_k;

    /* Start for DataStoreMemory: '<S4>/Data Store Memory' */
    for (STM32_Nucleo_Programming_B.i = 0; STM32_Nucleo_Programming_B.i < 64;
         STM32_Nucleo_Programming_B.i++) {
      ToF[STM32_Nucleo_Programming_B.i] =
        STM32_Nucleo_Programming_P.DataStoreMemory_InitialValue_g;
    }

    /* End of Start for DataStoreMemory: '<S4>/Data Store Memory' */

    /* user code (Initialize function Body) */

    /* System '<Root>' */
    MX_USART2_UART_Init();

    /* SystemInitialize for Atomic SubSystem: '<S3>/Magnetometer initialization' */
    /* Start for MATLABSystem: '<S5>/Magnetometer initialization' */
    STM32_Nucleo_Programming_DW.obj_m.DefaultMaximumBusSpeedInHz = 400000.0;
    STM32_Nucleo_Programming_DW.obj_m.matlabCodegenIsDeleted = false;
    obj_2 = &STM32_Nucleo_Programming_DW.obj_m;
    STM32_Nucleo_Programming_DW.obj_m.isInitialized = 1;
    STM32_Nucleo_Programming_B.ModeType = MW_I2C_MASTER;
    STM32_Nucleo_Programming_B.i2cname = 0;
    obj_2->MW_I2C_HANDLE = MW_I2C_Open(STM32_Nucleo_Programming_B.i2cname,
      STM32_Nucleo_Programming_B.ModeType);
    STM32_Nucleo_Programming_DW.obj_m.BusSpeed = 100000U;
    MW_I2C_SetBusSpeed(STM32_Nucleo_Programming_DW.obj_m.MW_I2C_HANDLE,
                       STM32_Nucleo_Programming_DW.obj_m.BusSpeed);
    STM32_Nucleo_Programming_DW.obj_m.isSetupComplete = true;

    /* End of SystemInitialize for SubSystem: '<S3>/Magnetometer initialization' */

    /* Start for MATLABSystem: '<S2>/LSM6DSL IMU Sensor' */
    STM32_Nucleo_Programming_DW.obj._pobj4.matlabCodegenIsDeleted = true;
    STM32_Nucleo_Programming_DW.obj.isInitialized = 0;
    STM32_Nucleo_Programming_DW.obj.matlabCodegenIsDeleted = false;
    STM32_Nucleo_Programming_DW.obj.SampleTime =
      STM32_Nucleo_Programming_P.LSM6DSLIMUSensor_SampleTime;
    STM32_Nucleo_P_SystemCore_setup(&STM32_Nucleo_Programming_DW.obj);

    /* Start for MATLABSystem: '<S3>/I2C Controller Read' */
    STM32_Nucleo_Programming_DW.obj_p.DefaultMaximumBusSpeedInHz = 400000.0;
    STM32_Nucleo_Programming_DW.obj_p.matlabCodegenIsDeleted = false;
    STM32_Nucleo_Programming_DW.obj_p.SampleTime =
      STM32_Nucleo_Programming_P.I2CControllerRead_SampleTime;
    obj = &STM32_Nucleo_Programming_DW.obj_p;
    STM32_Nucleo_Programming_DW.obj_p.isInitialized = 1;
    STM32_Nucleo_Programming_B.ModeType = MW_I2C_MASTER;
    STM32_Nucleo_Programming_B.i2cname = 0;
    obj->MW_I2C_HANDLE = MW_I2C_Open(STM32_Nucleo_Programming_B.i2cname,
      STM32_Nucleo_Programming_B.ModeType);
    STM32_Nucleo_Programming_DW.obj_p.BusSpeed = 100000U;
    MW_I2C_SetBusSpeed(STM32_Nucleo_Programming_DW.obj_p.MW_I2C_HANDLE,
                       STM32_Nucleo_Programming_DW.obj_p.BusSpeed);
    STM32_Nucleo_Programming_DW.obj_p.isSetupComplete = true;

    /* Start for MATLABSystem: '<Root>/Blue Button' */
    STM32_Nucleo_Programming_DW.obj_c.matlabCodegenIsDeleted = false;
    STM32_Nucleo_Programming_DW.obj_c.SampleTime =
      STM32_Nucleo_Programming_P.BlueButton_SampleTime;
    obj_0 = &STM32_Nucleo_Programming_DW.obj_c;
    STM32_Nucleo_Programming_DW.obj_c.isInitialized = 1;
    STM32_Nucleo_Programming_B.i2cname = PC_13;
    obj_0->MW_DIGITALIO_HANDLE = MW_digitalIO_open
      (STM32_Nucleo_Programming_B.i2cname, 0);
    STM32_Nucleo_Programming_DW.obj_c.isSetupComplete = true;

    /* Start for MATLABSystem: '<Root>/Green LED' */
    STM32_Nucleo_Programming_DW.obj_k.matlabCodegenIsDeleted = false;
    obj_1 = &STM32_Nucleo_Programming_DW.obj_k;
    STM32_Nucleo_Programming_DW.obj_k.isInitialized = 1;
    STM32_Nucleo_Programming_B.i2cname = PA_5;
    obj_1->MW_DIGITALIO_HANDLE = MW_digitalIO_open
      (STM32_Nucleo_Programming_B.i2cname, 1);
    STM32_Nucleo_Programming_DW.obj_k.isSetupComplete = true;

    /* Outputs for Atomic SubSystem: '<S3>/Magnetometer initialization' */
    /* MATLABSystem: '<S5>/Magnetometer initialization' incorporates:
     *  Constant: '<S5>/Constant'
     */
    b_SwappedDataBytes[0] = 96U;
    b_SwappedDataBytes[1] = STM32_Nucleo_Programming_P.Constant_Value;
    MW_I2C_MasterWrite(STM32_Nucleo_Programming_DW.obj_m.MW_I2C_HANDLE, 30U,
                       &b_SwappedDataBytes[0], 2U, false, false);

    /* End of Outputs for SubSystem: '<S3>/Magnetometer initialization' */

    /* user code (Update function Body) */

    /* System '<Root>' */
    Dump_IKS02A1();
  }
}

/* Model terminate function */
void STM32_Nucleo_Programming_terminate(void)
{
  b_lsm6dsl_STM32_Nucleo_Progra_T *obj;

  /* Terminate for MATLABSystem: '<S2>/LSM6DSL IMU Sensor' */
  if (!STM32_Nucleo_Programming_DW.obj.matlabCodegenIsDeleted) {
    STM32_Nucleo_Programming_DW.obj.matlabCodegenIsDeleted = true;
  }

  obj = &STM32_Nucleo_Programming_DW.obj._pobj4;
  if (!obj->matlabCodegenIsDeleted) {
    obj->matlabCodegenIsDeleted = true;
    if (obj->isInitialized == 1) {
      obj->isInitialized = 2;
    }
  }

  /* End of Terminate for MATLABSystem: '<S2>/LSM6DSL IMU Sensor' */

  /* Terminate for MATLABSystem: '<S3>/I2C Controller Read' */
  if (!STM32_Nucleo_Programming_DW.obj_p.matlabCodegenIsDeleted) {
    STM32_Nucleo_Programming_DW.obj_p.matlabCodegenIsDeleted = true;
    if ((STM32_Nucleo_Programming_DW.obj_p.isInitialized == 1) &&
        STM32_Nucleo_Programming_DW.obj_p.isSetupComplete) {
      MW_I2C_Close(STM32_Nucleo_Programming_DW.obj_p.MW_I2C_HANDLE);
    }
  }

  /* End of Terminate for MATLABSystem: '<S3>/I2C Controller Read' */

  /* Terminate for MATLABSystem: '<Root>/Blue Button' */
  if (!STM32_Nucleo_Programming_DW.obj_c.matlabCodegenIsDeleted) {
    STM32_Nucleo_Programming_DW.obj_c.matlabCodegenIsDeleted = true;
    if ((STM32_Nucleo_Programming_DW.obj_c.isInitialized == 1) &&
        STM32_Nucleo_Programming_DW.obj_c.isSetupComplete) {
      MW_digitalIO_close(STM32_Nucleo_Programming_DW.obj_c.MW_DIGITALIO_HANDLE);
    }
  }

  /* End of Terminate for MATLABSystem: '<Root>/Blue Button' */

  /* Terminate for MATLABSystem: '<Root>/Green LED' */
  if (!STM32_Nucleo_Programming_DW.obj_k.matlabCodegenIsDeleted) {
    STM32_Nucleo_Programming_DW.obj_k.matlabCodegenIsDeleted = true;
    if ((STM32_Nucleo_Programming_DW.obj_k.isInitialized == 1) &&
        STM32_Nucleo_Programming_DW.obj_k.isSetupComplete) {
      MW_digitalIO_close(STM32_Nucleo_Programming_DW.obj_k.MW_DIGITALIO_HANDLE);
    }
  }

  /* End of Terminate for MATLABSystem: '<Root>/Green LED' */

  /* Terminate for Atomic SubSystem: '<S3>/Magnetometer initialization' */
  /* Terminate for MATLABSystem: '<S5>/Magnetometer initialization' */
  if (!STM32_Nucleo_Programming_DW.obj_m.matlabCodegenIsDeleted) {
    STM32_Nucleo_Programming_DW.obj_m.matlabCodegenIsDeleted = true;
    if ((STM32_Nucleo_Programming_DW.obj_m.isInitialized == 1) &&
        STM32_Nucleo_Programming_DW.obj_m.isSetupComplete) {
      MW_I2C_Close(STM32_Nucleo_Programming_DW.obj_m.MW_I2C_HANDLE);
    }
  }

  /* End of Terminate for MATLABSystem: '<S5>/Magnetometer initialization' */
  /* End of Terminate for SubSystem: '<S3>/Magnetometer initialization' */
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
