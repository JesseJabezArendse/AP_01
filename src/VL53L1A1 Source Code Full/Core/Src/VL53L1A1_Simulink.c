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
// #include "stm32f4xx_nucleo.h"
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

int32_t tof_fsr;
float tof_odr;

extern RANGING_SENSOR_Result_t TOF_left;
extern RANGING_SENSOR_Result_t TOF_centre;
extern RANGING_SENSOR_Result_t TOF_right;
extern RANGING_SENSOR_ProfileConfig_t Profile;
RANGING_SENSOR_ROIConfig_t ROI;

extern uint32_t counter;
extern UART_HandleTypeDef huart2;

static int32_t status = 0;
static uint8_t ToF_Present[RANGING_SENSOR_INSTANCES_NBR] = {0};
volatile uint8_t ToF_EventDetected = 0;

const uint8_t L1_BUFFER_SIZE = 60;
RANGING_SENSOR_Result_t TOF_left_array[60];
RANGING_SENSOR_Result_t TOF_centre_array[60];
RANGING_SENSOR_Result_t TOF_right_array[60];
// mask options from BSP/Components/<IC_reg.h>

// Functions /////////////////////////////////////////////

void initVL53L1A1(){
    uint8_t device;
    uint16_t i2c_addr;
    uint32_t id;

    /* put all the devices in shutdown mode */
    for (device = 0; device < RANGING_SENSOR_INSTANCES_NBR; device++)
    {
        VL53L1A2_RANGING_SENSOR_SetPowerMode(device, RANGING_SENSOR_POWERMODE_OFF);
    }

    /* power on the devices one at a time, initialize them and change their address.
   * once the address is updated, the communication with the devices is checked
   * reading its ID.
   */
    for (device = 0; device < RANGING_SENSOR_INSTANCES_NBR; device++)
    {
        VL53L1A2_RANGING_SENSOR_SetPowerMode(device, RANGING_SENSOR_POWERMODE_ON);
        status = VL53L1A2_RANGING_SENSOR_Init(device);

        /* 0: not detected, 1: detected */
        ToF_Present[device] = (status != BSP_ERROR_NONE) ? 0 : 1;

        /* skip this device if init not successful */
        if (ToF_Present[device] == 0) { continue; }

        /* left: 0x54, center: 0x56, right: 0x58 */
        i2c_addr = (RANGING_SENSOR_VL53L1CB_ADDRESS + (device + 1) * 2);
        VL53L1A2_RANGING_SENSOR_SetAddress(device, i2c_addr);

        /* check the communication with the device reading the ID */
        VL53L1A2_RANGING_SENSOR_ReadID(device, &id);
    }

    
    Profile.TimingBudget = 1000/tof_odr;
    Profile.Frequency = 0;
    Profile.EnableAmbient = 1; /* Enable: 1, Disable: 0 */
    Profile.EnableSignal = 1; /* Enable: 1, Disable: 0 */

    ROI.TopLeftX  = 8; 
    ROI.TopLeftY  = 12; 
    ROI.BotRightX = 12;
    ROI.BotRightY = 8;

    VL53L1A2_RANGING_SENSOR_ConfigProfile(INSTANCE_TOF_LEFT   , &Profile);
    VL53L1A2_RANGING_SENSOR_ConfigProfile(INSTANCE_TOF_CENTRE , &Profile);
    VL53L1A2_RANGING_SENSOR_ConfigProfile(INSTANCE_TOF_RIGHT  , &Profile);

    VL53L1A2_RANGING_SENSOR_ConfigROI(INSTANCE_TOF_LEFT   , &ROI);
    VL53L1A2_RANGING_SENSOR_ConfigROI(INSTANCE_TOF_CENTRE , &ROI);
    VL53L1A2_RANGING_SENSOR_ConfigROI(INSTANCE_TOF_RIGHT  , &ROI);

    VL53L1A2_RANGING_SENSOR_Start(INSTANCE_TOF_LEFT   , RS_MODE_ASYNC_CONTINUOUS);
    VL53L1A2_RANGING_SENSOR_Start(INSTANCE_TOF_CENTRE , RS_MODE_ASYNC_CONTINUOUS);
    VL53L1A2_RANGING_SENSOR_Start(INSTANCE_TOF_RIGHT  , RS_MODE_ASYNC_CONTINUOUS);
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

void selectBestTOF(RANGING_SENSOR_Result_t *array) {
    uint8_t bestIndex = 0;
    uint8_t bestSubIndex = 0;
    float_t bestRatio = 0.0f;

    for (uint8_t i = 0; i < 60; i++) {
        for (uint8_t j = 0; j < 4; j++) { 
            float_t ratio = 0; 
            if (array[i].ZoneResult[j].Signal[j] != 0 ) {ratio =  array[i].ZoneResult[j].Signal[j] / array[i].ZoneResult[j].Ambient[j];}
            if (ratio > bestRatio) {
                bestRatio = ratio;
                bestIndex = i;
                bestSubIndex = j;
            }
        }
    }
    array[0].ZoneResult[0].Ambient[0]  = array[bestIndex].ZoneResult[bestSubIndex].Ambient[bestSubIndex]  ;
    array[0].ZoneResult[0].Distance[0] = array[bestIndex].ZoneResult[bestSubIndex].Distance[bestSubIndex] ;
    array[0].ZoneResult[0].Signal[0]   = array[bestIndex].ZoneResult[bestSubIndex].Signal[bestSubIndex]   ;
    
}

RANGING_SENSOR_Result_t sub_TOF_left;
RANGING_SENSOR_Result_t sub_TOF_centre;
RANGING_SENSOR_Result_t sub_TOF_right;
RANGING_SENSOR_Result_t temp_TOF_left;
RANGING_SENSOR_Result_t temp_TOF_centre;
RANGING_SENSOR_Result_t temp_TOF_right;


uint8_t buff_counter = 0;

uint8_t isItWorthy(RANGING_SENSOR_Result_t *sub_TOF){
    if (sub_TOF->ZoneResult[0].Status[sub_TOF->ZoneResult->NumberOfTargets-1] == 0){
        return 1;
    }
    else{
        return 0;
    }
}

void getVL53L1A1(){
    VL53L1A2_RANGING_SENSOR_GetDistance(INSTANCE_TOF_LEFT   , &temp_TOF_left   );
    VL53L1A2_RANGING_SENSOR_GetDistance(INSTANCE_TOF_CENTRE , &temp_TOF_centre );
    VL53L1A2_RANGING_SENSOR_GetDistance(INSTANCE_TOF_RIGHT  , &temp_TOF_right  );

    for (int i = 0 ; i < 4 ; i++){
        if (((temp_TOF_left   .ZoneResult[0]) .Status  [i]) == 0){
            TOF_left = temp_TOF_left;
        }
        if (((temp_TOF_left   .ZoneResult[0]) .Status  [i]) == 255){
            TOF_left = temp_TOF_left;
        }
    }

    for (int i = 0 ; i < 4 ; i++){
        if (((temp_TOF_centre   .ZoneResult[0]) .Status  [i]) == 0){
            TOF_centre = temp_TOF_centre;
        }
        if (((temp_TOF_centre   .ZoneResult[0]) .Status  [i]) == 255){
            TOF_centre = temp_TOF_centre;
        }
    }

    for (int i = 0 ; i < 4 ; i++){
        if (((temp_TOF_right   .ZoneResult[0]) .Status  [i]) == 0){
            TOF_right = temp_TOF_right;
        }
        if (((temp_TOF_right   .ZoneResult[0]) .Status  [i]) == 255){
            TOF_right = temp_TOF_right;
        }
    }
}

void receivedFromSimulink(uint8_t* bigBuffer){
    tof_fsr = bytesToInt32(bigBuffer[3  + 0] , bigBuffer[3  + 1] , bigBuffer[3  + 2] ,  bigBuffer[3  + 3] );
    tof_odr = bytesToFloat(bigBuffer[3  + 4] , bigBuffer[3  + 5] , bigBuffer[3  + 6] ,  bigBuffer[3  + 7] );

    calibrate_VL53L1A1();
}

void sendToSimulink(){

    HAL_UART_Transmit(&huart2, (uint8_t *) &header           ,3 , HAL_MAX_DELAY);
    
    HAL_UART_Transmit(&huart2, (uint32_t *) &((TOF_left   .ZoneResult[0]) .Distance  [TOF_left.ZoneResult[0].NumberOfTargets -1])  , 4 , HAL_MAX_DELAY);
    HAL_UART_Transmit(&huart2, (float_t *)  &((TOF_left   .ZoneResult[0]) .Ambient   [TOF_left.ZoneResult[0].NumberOfTargets -1])  , 4 , HAL_MAX_DELAY);
    HAL_UART_Transmit(&huart2, (float_t *)  &((TOF_left   .ZoneResult[0]) .Signal    [TOF_left.ZoneResult[0].NumberOfTargets -1])  , 4 , HAL_MAX_DELAY);
    HAL_UART_Transmit(&huart2, (float_t *)  &((TOF_left   .ZoneResult[0]) .Status    [TOF_left.ZoneResult[0].NumberOfTargets -1])  , 4 , HAL_MAX_DELAY);

    HAL_UART_Transmit(&huart2, (uint32_t *) &((TOF_centre .ZoneResult[0]) .Distance  [TOF_centre.ZoneResult[0].NumberOfTargets -1])  , 4 , HAL_MAX_DELAY);
    HAL_UART_Transmit(&huart2, (float_t *)  &((TOF_centre .ZoneResult[0]) .Ambient   [TOF_centre.ZoneResult[0].NumberOfTargets -1])  , 4 , HAL_MAX_DELAY);
    HAL_UART_Transmit(&huart2, (float_t *)  &((TOF_centre .ZoneResult[0]) .Signal    [TOF_centre.ZoneResult[0].NumberOfTargets -1])  , 4 , HAL_MAX_DELAY);
    HAL_UART_Transmit(&huart2, (float_t *)  &((TOF_centre .ZoneResult[0]) .Status    [TOF_centre.ZoneResult[0].NumberOfTargets -1])  , 4 , HAL_MAX_DELAY);

    HAL_UART_Transmit(&huart2, (uint32_t *) &((TOF_right  .ZoneResult[0]) .Distance  [TOF_right.ZoneResult[0].NumberOfTargets -1])  , 4 , HAL_MAX_DELAY);
    HAL_UART_Transmit(&huart2, (float_t *)  &((TOF_right  .ZoneResult[0]) .Ambient   [TOF_right.ZoneResult[0].NumberOfTargets -1])  , 4 , HAL_MAX_DELAY);
    HAL_UART_Transmit(&huart2, (float_t *)  &((TOF_right  .ZoneResult[0]) .Signal    [TOF_right.ZoneResult[0].NumberOfTargets -1])  , 4 , HAL_MAX_DELAY);
    HAL_UART_Transmit(&huart2, (float_t *)  &((TOF_right  .ZoneResult[0]) .Status    [TOF_right.ZoneResult[0].NumberOfTargets -1])  , 4 , HAL_MAX_DELAY);

    HAL_UART_Transmit(&huart2, (uint32_t *) &counter         ,4 , HAL_MAX_DELAY);
    HAL_UART_Transmit(&huart2, (uint8_t *) &terminator       ,3 , HAL_MAX_DELAY);
}


// functions from BSP/<boards>
// parameters from BSP/<boards>
void calibrate_VL53L1A1(){

    if (tof_fsr == 1){
        Profile.RangingProfile = RS_MULTI_TARGET_SHORT_RANGE;
    }
    if (tof_fsr == 2){
        Profile.RangingProfile = RS_MULTI_TARGET_MEDIUM_RANGE;
    }
    if (tof_fsr == 3){
        Profile.RangingProfile = RS_MULTI_TARGET_LONG_RANGE;
    }
}