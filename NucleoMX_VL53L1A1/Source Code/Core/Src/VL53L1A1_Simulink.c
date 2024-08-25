///////////////////////////////////////////////////////////
// Project : STM32 Nucleo + VL53L1A1
// Abbreviation : AP_01
// Microcontroller: STM32 Nucleo F411RE
// Made For : Dr. Amir Patel
// @author: Jesse Jabez Arendse
// @modified: 24/08/2024
///////////////////////////////////////////////////////////


// Includes ///////////////////////////////////////////////
#include "53l1a2_ranging_sensor.h"
#include "53l1a2_conf.h"
#include "stm32f4xx_nucleo.h"
#include "main.h"

#include "stdio.h"
#include <stdint.h>
#include <string.h>  // For memcpy
#include <stdio.h>


// Project Parent Variables ///////////////////////////////
const uint8_t* header = 'A_J';
const uint8_t* terminator = 'J_A';


// Project Specific Variables /////////////////////////////
#define INSTANCE_TOF_LEFT    0 
#define INSTANCE_TOF_CENTRE  1 
#define INSTANCE_TOF_RIGHT   2 

#define TIMING_BUDGET (30U) /* 16 ms < TimingBudget < 500 ms */
#define POLLING_PERIOD (250U) /* refresh rate for polling mode (ms, shall be consistent with TimingBudget value) */


extern RANGING_SENSOR_Result_t TOF_left;
extern RANGING_SENSOR_Result_t TOF_centre;
extern RANGING_SENSOR_Result_t TOF_right;


// mask options from BSP/Components/<IC_reg.h>

// Functions /////////////////////////////////////////////

void initVL53L1A1(){
    RANGING_SENSOR_ProfileConfig_t Profile;
    Profile.RangingProfile = RS_MULTI_TARGET_MEDIUM_RANGE;
    Profile.TimingBudget = TIMING_BUDGET;
    Profile.Frequency = 0;
    Profile.EnableAmbient = 1; /* Enable: 1, Disable: 0 */
    Profile.EnableSignal = 1; /* Enable: 1, Disable: 0 */

    VL53L1A2_RANGING_SENSOR_ConfigProfile(INSTANCE_TOF_LEFT   , &Profile);
    VL53L1A2_RANGING_SENSOR_ConfigProfile(INSTANCE_TOF_CENTRE , &Profile);
    VL53L1A2_RANGING_SENSOR_ConfigProfile(INSTANCE_TOF_RIGHT  , &Profile);
    VL53L1A2_RANGING_SENSOR_Start(INSTANCE_TOF_LEFT   , RS_MODE_BLOCKING_CONTINUOUS);
    VL53L1A2_RANGING_SENSOR_Start(INSTANCE_TOF_CENTRE , RS_MODE_BLOCKING_CONTINUOUS);
    VL53L1A2_RANGING_SENSOR_Start(INSTANCE_TOF_RIGHT  , RS_MODE_BLOCKING_CONTINUOUS);
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

void getVL53L1A1(){
    VL53L1A2_RANGING_SENSOR_GetDistance(INSTANCE_TOF_LEFT   , &TOF_left   );
    VL53L1A2_RANGING_SENSOR_GetDistance(INSTANCE_TOF_CENTRE , &TOF_centre );
    VL53L1A2_RANGING_SENSOR_GetDistance(INSTANCE_TOF_RIGHT  , &TOF_right  );
}

void receivedFromSimulink(uint8_t* bigBuffer){
    // accel1_fsr = bytesToInt32(bigBuffer[3  + 0] , bigBuffer[3  + 1] , bigBuffer[3  + 2] ,  bigBuffer[3  + 3] );
    // calibrateVL53L1A1();
}

void sendToSimulink(){

    HAL_UART_Transmit(&hcom_uart[0], (uint8_t *) &header           ,3 , HAL_MAX_DELAY);
    HAL_UART_Transmit(&hcom_uart[0], (uint32_t *) &((TOF_left   .ZoneResult[0]) .Distance  [0])  , 4 , HAL_MAX_DELAY);
    HAL_UART_Transmit(&hcom_uart[0], (float_t *)  &((TOF_left   .ZoneResult[0]) .Ambient   [0])  , 4 , HAL_MAX_DELAY);
    HAL_UART_Transmit(&hcom_uart[0], (float_t *)  &((TOF_left   .ZoneResult[0]) .Signal    [0])  , 4 , HAL_MAX_DELAY);
    HAL_UART_Transmit(&hcom_uart[0], (uint32_t *) &((TOF_centre .ZoneResult[0]) .Distance  [0])  , 4 , HAL_MAX_DELAY);
    HAL_UART_Transmit(&hcom_uart[0], (float_t *)  &((TOF_centre .ZoneResult[0]) .Ambient   [0])  , 4 , HAL_MAX_DELAY);
    HAL_UART_Transmit(&hcom_uart[0], (float_t *)  &((TOF_centre .ZoneResult[0]) .Signal    [0])  , 4 , HAL_MAX_DELAY);
    HAL_UART_Transmit(&hcom_uart[0], (uint32_t *) &((TOF_right  .ZoneResult[0]) .Distance  [0])  , 4 , HAL_MAX_DELAY);
    HAL_UART_Transmit(&hcom_uart[0], (float_t *)  &((TOF_right  .ZoneResult[0]) .Ambient   [0])  , 4 , HAL_MAX_DELAY);
    HAL_UART_Transmit(&hcom_uart[0], (float_t *)  &((TOF_right  .ZoneResult[0]) .Signal    [0])  , 4 , HAL_MAX_DELAY);
    HAL_UART_Transmit(&hcom_uart[0], (uint8_t *) &terminator       ,3 , HAL_MAX_DELAY);
}






// functions from BSP/<boards>
// parameters from BSP/<boards>
void calibrateVL53L1A1(){

    // Accel1
    // IKS02A1_MOTION_SENSOR_SetFullScale      (INSTANCE_ISM330DHCX,MOTION_ACCELERO,accel1_fsr);
    // IKS02A1_MOTION_SENSOR_SetOutputDataRate (INSTANCE_ISM330DHCX,MOTION_ACCELERO,accel1_odr);

}