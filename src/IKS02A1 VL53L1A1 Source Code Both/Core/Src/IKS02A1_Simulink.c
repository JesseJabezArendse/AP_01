///////////////////////////////////////////////////////////
// Project : STM32 Nucleo + IKS02A1
// Abbreviation : AP_01
// Microcontroller: STM32 Nucleo F411RE
// Made For : Dr. Amir Patel
// @author: Jesse Jabez Arendse
// @modified: 24/08/2024
///////////////////////////////////////////////////////////


// Includes ///////////////////////////////////////////////
#include "iks02a1_motion_sensors.h"
// #include "stm32f4xx_nucleo.h"
#include "iks02a1_conf.h"
#include "main.h"

#include "stdio.h"
#include <stdint.h>
#include <string.h>  // For memcpy
#include <stdio.h>


// Project Parent Variables ///////////////////////////////


// extern UART_HandleTypeDef hcom_uart[1];   // com port

uint32_t tim2_psc;
uint32_t tim2_arr;

// Project Specific Variables /////////////////////////////

#define INSTANCE_ISM330DHCX IKS02A1_ISM330DHCX_0
#define INSTANCE_IIS2DLPC   IKS02A1_IIS2DLPC_0
#define INSTANCE_IIS2MDC    IKS02A1_IIS2MDC_0

extern UART_HandleTypeDef huart2;
extern IKS02A1_MOTION_SENSOR_Axes_t accel1_axis;
extern IKS02A1_MOTION_SENSOR_Axes_t gyro_axis;
extern IKS02A1_MOTION_SENSOR_Axes_t accel2_axis;
extern IKS02A1_MOTION_SENSOR_Axes_t mag_axis;
extern int32_t counter;

// mask options from BSP/Components/<IC_reg.h>
int32_t accel1_fsr;
float accel1_odr;
int32_t gyro_fsr;
float gyro_odr;

int32_t accel2_fsr;
float accel2_odr;

float mag_odr;

float fastestODR;

uint8_t temperature_raw_l;
uint8_t temperature_raw_h;
float_t temperature;

// Functions /////////////////////////////////////////////


void getTemperature(){
    HAL_I2C_Mem_Read(&hi2c1 , IIS2DLPC_I2C_ADD_H , IIS2DLPC_OUT_T_L , 1 , &temperature_raw_l , 1 , HAL_MAX_DELAY);
    HAL_I2C_Mem_Read(&hi2c1 , IIS2DLPC_I2C_ADD_H , IIS2DLPC_OUT_T_H , 1 , &temperature_raw_h , 1 , HAL_MAX_DELAY);
    int16_t rawTemp = (int16_t)((temperature_raw_h << 8) | temperature_raw_l); // Combine low and high bytes
    temperature = iis2dlpc_from_lsb_to_celsius(rawTemp);
}

float bytesToFloat(uint8_t byte1 , uint8_t byte2 , uint8_t byte3 , uint8_t byte4) {
    float result;
    uint8_t bytes[4] = {byte1,byte2,byte3,byte4};

    // Use memcpy to copy the 4 bytes into a float (this preserves the binary representation)
    memcpy(&result, bytes, sizeof(float));

    return result;
}

int32_t bytesToInt32(uint8_t byte1 , uint8_t byte2 , uint8_t byte3 , uint8_t byte4)  {
    int32_t result;
    uint8_t bytes[4] = {byte1,byte2,byte3,byte4};

    // Use memcpy to copy the 4 bytes into a int32_t (this preserves the binary representation)
    memcpy(&result, bytes, sizeof(int32_t));

    return result;
}

void initIKS02A1(){

  IKS02A1_MOTION_SENSOR_Init(IKS02A1_ISM330DHCX_0, MOTION_ACCELERO | MOTION_GYRO);

  IKS02A1_MOTION_SENSOR_Init(IKS02A1_IIS2DLPC_0, MOTION_ACCELERO);

  IKS02A1_MOTION_SENSOR_Init(IKS02A1_IIS2MDC_0, MOTION_MAGNETO);
  
}

void getIKS02A1(){
    while (IKS02A1_MOTION_SENSOR_GetAxes(IKS02A1_ISM330DHCX_0, MOTION_ACCELERO , &accel1_axis) != BSP_ERROR_NONE){}
    while (IKS02A1_MOTION_SENSOR_GetAxes(IKS02A1_ISM330DHCX_0, MOTION_GYRO     , &gyro_axis  ) != BSP_ERROR_NONE){}
    while (IKS02A1_MOTION_SENSOR_GetAxes(IKS02A1_IIS2DLPC_0,   MOTION_ACCELERO , &accel2_axis) != BSP_ERROR_NONE){}
    while (IKS02A1_MOTION_SENSOR_GetAxes(IKS02A1_IIS2MDC_0,    MOTION_MAGNETO  , &mag_axis   ) != BSP_ERROR_NONE){}
    getTemperature();
}

void initial_calibrate(){
    // Accel1
    // IKS02A1_MOTION_SENSOR_SetFullScale      (INSTANCE_ISM330DHCX,MOTION_ACCELERO,accel1_fsr);
    IKS02A1_MOTION_SENSOR_SetOutputDataRate (INSTANCE_ISM330DHCX,MOTION_ACCELERO,208.0f);
    // Gyro1
    // IKS02A1_MOTION_SENSOR_SetFullScale      (INSTANCE_ISM330DHCX,MOTION_GYRO,gyro_fsr);
    IKS02A1_MOTION_SENSOR_SetOutputDataRate (INSTANCE_ISM330DHCX,MOTION_GYRO,208.0f);

    // Accel2
    // IKS02A1_MOTION_SENSOR_SetFullScale      (INSTANCE_IIS2DLPC,MOTION_ACCELERO,accel2_fsr);
    IKS02A1_MOTION_SENSOR_SetOutputDataRate (INSTANCE_IIS2DLPC,MOTION_ACCELERO,accel2_odr);

    // Magneto
    IKS02A1_MOTION_SENSOR_SetOutputDataRate (INSTANCE_IIS2MDC,MOTION_MAGNETO,mag_odr);
}

// functions from BSP/<boards>
// parameters from BSP/<boards>
void calibrate_IKS02A1(){

    // Accel1
    IKS02A1_MOTION_SENSOR_SetFullScale      (INSTANCE_ISM330DHCX,MOTION_ACCELERO,accel1_fsr);
    IKS02A1_MOTION_SENSOR_SetOutputDataRate (INSTANCE_ISM330DHCX,MOTION_ACCELERO,accel1_odr);
    // Gyro1
    IKS02A1_MOTION_SENSOR_SetFullScale      (INSTANCE_ISM330DHCX,MOTION_GYRO,gyro_fsr);
    IKS02A1_MOTION_SENSOR_SetOutputDataRate (INSTANCE_ISM330DHCX,MOTION_GYRO,gyro_odr);

    // Accel2
    IKS02A1_MOTION_SENSOR_SetFullScale      (INSTANCE_IIS2DLPC,MOTION_ACCELERO,accel2_fsr);
    IKS02A1_MOTION_SENSOR_SetOutputDataRate (INSTANCE_IIS2DLPC,MOTION_ACCELERO,accel2_odr);

    // Magneto
    IKS02A1_MOTION_SENSOR_SetOutputDataRate (INSTANCE_IIS2MDC,MOTION_MAGNETO,mag_odr);

}