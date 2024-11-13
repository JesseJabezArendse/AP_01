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
#include "stm32f4xx_nucleo.h"
#include "iks02a1_conf.h"
#include "main.h"

#include "stdio.h"
#include <stdint.h>
#include <string.h>  // For memcpy
#include <stdio.h>


// Project Parent Variables ///////////////////////////////
const uint8_t* header = 'A_J';
const uint8_t* terminator = 'J_A';

// extern UART_HandleTypeDef hcom_uart[1];   // com port

uint32_t tim2_psc;
uint32_t tim2_arr;

// Project Specific Variables /////////////////////////////

#define INSTANCE_ISM330DHCX IKS02A1_ISM330DHCX_0
#define INSTANCE_IIS2DLPC   IKS02A1_IIS2DLPC_0
#define INSTANCE_IIS2MDC    IKS02A1_IIS2MDC_0

extern IKS02A1_MOTION_SENSOR_Axes_t accel1_axis;
extern IKS02A1_MOTION_SENSOR_Axes_t gyro_axis;
extern IKS02A1_MOTION_SENSOR_Axes_t accel2_axis;
extern IKS02A1_MOTION_SENSOR_Axes_t mag_axis;

// mask options from BSP/Components/<IC_reg.h>
int32_t accel1_fsr;
float accel1_odr;
int32_t gyro_fsr;
float gyro_odr;

int32_t accel2_fsr;
float accel2_odr;

float mag_odr;

// Functions /////////////////////////////////////////////

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

     /* Initialize LED */
  BSP_LED_Init(LED2);

  /* Initialize button */
  BSP_PB_Init(BUTTON_KEY, BUTTON_MODE_EXTI);

  /* Check what is the Push Button State when the button is not pressed. It can change across families */
  static int32_t PushButtonState = GPIO_PIN_RESET;
  PushButtonState = (BSP_PB_GetState(BUTTON_KEY)) ?  0 : 1;

  /* Initialize Virtual COM Port */
  BSP_COM_Init(COM1);

  IKS02A1_MOTION_SENSOR_Init(IKS02A1_ISM330DHCX_0, MOTION_ACCELERO | MOTION_GYRO);

  IKS02A1_MOTION_SENSOR_Init(IKS02A1_IIS2DLPC_0, MOTION_ACCELERO);

  IKS02A1_MOTION_SENSOR_Init(IKS02A1_IIS2MDC_0, MOTION_MAGNETO);

    
    int32_t ret = IKS02A1_MOTION_SENSOR_Init(IKS02A1_ISM330DHCX_0, MOTION_ACCELERO | MOTION_GYRO);
    if (IKS02A1_MOTION_SENSOR_Init(IKS02A1_IIS2DLPC_0, MOTION_ACCELERO) != BSP_ERROR_NONE) {Error_Handler();}
    if (IKS02A1_MOTION_SENSOR_Init(IKS02A1_IIS2MDC_0, MOTION_MAGNETO) != BSP_ERROR_NONE) {Error_Handler();}
}

void getIKS02A1(){
    while (IKS02A1_MOTION_SENSOR_GetAxes(IKS02A1_ISM330DHCX_0, MOTION_ACCELERO , &accel1_axis) != BSP_ERROR_NONE){}
    while (IKS02A1_MOTION_SENSOR_GetAxes(IKS02A1_ISM330DHCX_0, MOTION_GYRO     , &gyro_axis  ) != BSP_ERROR_NONE){}
    while (IKS02A1_MOTION_SENSOR_GetAxes(IKS02A1_IIS2DLPC_0,   MOTION_ACCELERO , &accel2_axis) != BSP_ERROR_NONE){}
    while (IKS02A1_MOTION_SENSOR_GetAxes(IKS02A1_IIS2MDC_0,    MOTION_MAGNETO  , &mag_axis   ) != BSP_ERROR_NONE){}
}

void receivedFromSimulink(uint8_t* bigBuffer){
    accel1_fsr = bytesToInt32(bigBuffer[3  + 0] , bigBuffer[3  + 1] , bigBuffer[3  + 2] ,  bigBuffer[3  + 3] );
    accel1_odr = bytesToFloat(bigBuffer[7  + 0] , bigBuffer[7  + 1] , bigBuffer[7  + 2] ,  bigBuffer[7  + 3] );
    gyro_fsr =   bytesToInt32(bigBuffer[11 + 0] , bigBuffer[11 + 1] , bigBuffer[11 + 2] ,  bigBuffer[11 + 3] );
    gyro_odr =   bytesToFloat(bigBuffer[15 + 0] , bigBuffer[15 + 1] , bigBuffer[15 + 2] ,  bigBuffer[15 + 3] );
    accel2_fsr = bytesToInt32(bigBuffer[19 + 0] , bigBuffer[19 + 1] , bigBuffer[19 + 2] ,  bigBuffer[19 + 3] );
    accel2_odr = bytesToFloat(bigBuffer[23 + 0] , bigBuffer[23 + 1] , bigBuffer[23 + 2] ,  bigBuffer[23 + 3] );
    mag_odr =    bytesToFloat(bigBuffer[27 + 0] , bigBuffer[27 + 1] , bigBuffer[27 + 2] ,  bigBuffer[27 + 3] );
    calibrate_IKS02A1();
}

void sendToSimulink(){

    HAL_UART_Transmit(&hcom_uart, (uint8_t *) &header           ,3 , HAL_MAX_DELAY);
    HAL_UART_Transmit(&hcom_uart, (int32_t *) &(accel1_axis.x)  ,4 , HAL_MAX_DELAY);
    HAL_UART_Transmit(&hcom_uart, (int32_t *) &accel1_axis.y    ,4 , HAL_MAX_DELAY);
    HAL_UART_Transmit(&hcom_uart, (int32_t *) &accel1_axis.z    ,4 , HAL_MAX_DELAY);
    HAL_UART_Transmit(&hcom_uart, (int32_t *) &gyro_axis.x      ,4 , HAL_MAX_DELAY);
    HAL_UART_Transmit(&hcom_uart, (int32_t *) &gyro_axis.y      ,4 , HAL_MAX_DELAY);
    HAL_UART_Transmit(&hcom_uart, (int32_t *) &gyro_axis.z      ,4 , HAL_MAX_DELAY);
    HAL_UART_Transmit(&hcom_uart, (int32_t *) &accel2_axis.x    ,4 , HAL_MAX_DELAY);
    HAL_UART_Transmit(&hcom_uart, (int32_t *) &accel2_axis.y    ,4 , HAL_MAX_DELAY);
    HAL_UART_Transmit(&hcom_uart, (int32_t *) &accel2_axis.z    ,4 , HAL_MAX_DELAY);
    HAL_UART_Transmit(&hcom_uart, (int32_t *) &mag_axis.x       ,4 , HAL_MAX_DELAY);
    HAL_UART_Transmit(&hcom_uart, (int32_t *) &mag_axis.y       ,4 , HAL_MAX_DELAY);
    HAL_UART_Transmit(&hcom_uart, (int32_t *) &mag_axis.z       ,4 , HAL_MAX_DELAY);
    HAL_UART_Transmit(&hcom_uart, (uint8_t *) &terminator       ,3 , HAL_MAX_DELAY);
}

void blueButtonPressed(){

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