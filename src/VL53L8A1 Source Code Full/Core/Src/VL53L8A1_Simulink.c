///////////////////////////////////////////////////////////
// Project : STM32 Nucleo + VL53L8A1
// Abbreviation : AP_01
// Microcontroller: STM32 Nucleo F411RE
// Made For : Dr. Amir Patel
// @author: Jesse Jabez Arendse
// @modified: 25/10/2024
///////////////////////////////////////////////////////////


// Includes ///////////////////////////////////////////////
#include "53l8a1_ranging_sensor.h"
#include "53l8a1_conf.h"
// #include "stm32f4xx_nucleo.h"
#include "main.h"

#include "stdio.h"
#include <stdint.h>
#include <string.h>  // For memcpy
#include <stdio.h>

#define TOF_INT_EXTI_PIN    (GPIO_PIN_4)
#define TOF_INT_EXTI_PORT   (GPIOA)

#define VL53L8A1_PWR_EN_C_PIN   (GPIO_PIN_7)
#define VL53L8A1_PWR_EN_C_PORT  (GPIOA)

#define VL53L8A1_LPn_C_PIN   (GPIO_PIN_0)
#define VL53L8A1_LPn_C_PORT  (GPIOB)
// Project Parent Variables ///////////////////////////////
const uint8_t* header = 'A_J';
const uint8_t* terminator = 'J_A';

extern UART_HandleTypeDef huart2;
extern uint32_t counter;

// Project Specific Variables /////////////////////////////
#define INSTANCE_TOF_CENTRE  VL53L8A1_DEV_CENTER

#define TIMING_BUDGET (30U) /* 16 ms < TimingBudget < 500 ms */
#define RANGING_FREQUENCY (10U) /* Ranging frequency Hz (shall be consistent with TimingBudget value) */
#define POLLING_PERIOD (1)

extern RANGING_SENSOR_Result_t TOF_centre;



// mask options from BSP/Components/<IC_reg.h>

// Functions /////////////////////////////////////////////
static RANGING_SENSOR_Capabilities_t Cap;
static int32_t status = 0;
RANGING_SENSOR_ProfileConfig_t Profile;

/*these are the default values for xtalk calibration, update them if needed */
uint16_t reflectance = 16; /* expressed in percent (%) - range 0% - 99% */
uint16_t cal_distance = 600; /* expressed in millimiters (mm) / range 600 mm - 3000 mm*/

int32_t tof_fsr;
float tof_odr;

uint32_t Id;
void initVL53L8A1(){

    HAL_GPIO_WritePin(VL53L8A1_PWR_EN_C_PORT, VL53L8A1_PWR_EN_C_PIN, GPIO_PIN_RESET);
    HAL_Delay(2);
    HAL_GPIO_WritePin(VL53L8A1_PWR_EN_C_PORT, VL53L8A1_PWR_EN_C_PIN, GPIO_PIN_SET);
    HAL_Delay(2);
    HAL_GPIO_WritePin(VL53L8A1_LPn_C_PORT, VL53L8A1_LPn_C_PIN, GPIO_PIN_RESET);
    HAL_Delay(2);
    HAL_GPIO_WritePin(VL53L8A1_LPn_C_PORT, VL53L8A1_LPn_C_PIN, GPIO_PIN_SET);
    HAL_Delay(2);

    status = VL53L8A1_RANGING_SENSOR_Init(VL53L8A1_DEV_CENTER);
    if (status != BSP_ERROR_NONE)
    {
        printf("VL53L8A1_RANGING_SENSOR_Init failed\n");
        while (1);
    }


    VL53L8A1_RANGING_SENSOR_ReadID(VL53L8A1_DEV_CENTER, &Id);
    VL53L8A1_RANGING_SENSOR_GetCapabilities(VL53L8A1_DEV_CENTER, &Cap);

    // printf("--- BEGIN XTALK CALIBRATION ---\n");
    // VL53L8A1_RANGING_SENSOR_XTalkCalibration(VL53L8A1_DEV_CENTER, reflectance, cal_distance);
    // printf("--- END OF XTALK CALIBRATION ---\n");


    // Profile.Frequency = RANGING_FREQUENCY; /* Ranging frequency Hz (shall be consistent with TimingBudget value) */
    Profile.RangingProfile = RS_PROFILE_8x8_CONTINUOUS;
    Profile.TimingBudget = TIMING_BUDGET;
    Profile.EnableAmbient = 1; /* Enable: 1, Disable: 0 */
    Profile.EnableSignal = 1; /* Enable: 1, Disable: 0 */

    VL53L8A1_RANGING_SENSOR_ConfigProfile(VL53L8A1_DEV_CENTER , &Profile);
    VL53L8A1_RANGING_SENSOR_Start(VL53L8A1_DEV_CENTER , RS_MODE_ASYNC_CONTINUOUS);
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

void getVL53L8A1(){
    VL53L8A1_RANGING_SENSOR_GetDistance(VL53L8A1_DEV_CENTER , &TOF_centre );
}

void sendToSimulink(){
    HAL_UART_Transmit(&huart2, (uint8_t *) &header           ,3 , HAL_MAX_DELAY);

    for (int i = 0 ; i < 8*8 ; i++){
        HAL_UART_Transmit(&huart2, (uint32_t *) &((TOF_centre .ZoneResult[i]) .Distance  [0])  , 4 , HAL_MAX_DELAY);
        HAL_UART_Transmit(&huart2, (float_t *)  &((TOF_centre .ZoneResult[i]) .Ambient   [0])  , 4 , HAL_MAX_DELAY);
        HAL_UART_Transmit(&huart2, (float_t *)  &((TOF_centre .ZoneResult[i]) .Signal    [0])  , 4 , HAL_MAX_DELAY);
        HAL_UART_Transmit(&huart2, (float_t *)  &((TOF_centre .ZoneResult[i]) .Status    [0])  , 4 , HAL_MAX_DELAY);
    }
    HAL_UART_Transmit(&huart2, (uint32_t *) &counter          ,4 , HAL_MAX_DELAY);
    HAL_UART_Transmit(&huart2, (uint8_t *) &terminator       ,3 , HAL_MAX_DELAY);
}

void receivedFromSimulink(uint8_t* bigBuffer){
    tof_fsr = bytesToInt32(bigBuffer[3  + 0] , bigBuffer[3  + 1] , bigBuffer[3  + 2] ,  bigBuffer[3  + 3] );
    tof_odr = bytesToFloat(bigBuffer[3  + 4] , bigBuffer[3  + 5] , bigBuffer[3  + 6] ,  bigBuffer[3  + 7] );

    calibrate_VL53L8A1();
}

// functions from BSP/<boards>
// parameters from BSP/<boards>
void calibrate_VL53L8A1(){
    Profile.RangingProfile = RS_PROFILE_8x8_CONTINUOUS;
    Profile.Frequency = tof_odr;
}