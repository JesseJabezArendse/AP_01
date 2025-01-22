///////////////////////////////////////////////////////////
// Project : STM32 Nucleo + VL53L1A1
// Abbreviation : AP_01
// Microcontroller: STM32 Nucleo F411RE
// Made For : Dr. Amir Patel
// @author: Jesse Jabez Arendse
// @modified: 20/01/2025
///////////////////////////////////////////////////////////


// Includes ///////////////////////////////////////////////
#include "main.h"

#include "stdio.h"
#include <stdint.h>
#include <string.h>  // For memcpy
#include <stdio.h>

#include "VL53L1X_api.h"
#include "X-NUCLEO-53L1A1.h"
#include <VL53L1A1_Simulink.h>

#define VL53L1X_POWER_ON  1
#define VL53L1X_POWER_OFF 0

#define TIMING_BUDGET (30U) /* 16 ms < TimingBudget < 500 ms */
#define POLLING_PERIOD (250U) /* refresh rate for polling mode (ms, shall be consistent with TimingBudget value) */


extern VL53L1_Result TOF_left_result;
extern VL53L1_Result TOF_centre_result;
extern VL53L1_Result TOF_right_result;

uint16_t VL53L1_address_default = 0x52;

uint8_t RangeStatus;
uint8_t dataReady;

extern int32_t tof_fsr;
extern float tof_odr;
extern uint16_t rangingProfile;
extern uint16_t timingBudget;
extern uint32_t pollingPeriod;

ROI_X = 4;
ROI_Y = 4;

uint16_t Distance;
uint16_t SignalRate;
uint16_t AmbientRate;
uint16_t SpadNum; 

extern uint32_t counter;
extern UART_HandleTypeDef huart2;


// Functions /////////////////////////////////////////////

void initVL53L1A1(int ToFNumber, uint8_t newToFAddress){
  int status = 0;
  uint8_t buffer[50];

  uint8_t byteData, sensorState=0;
  uint16_t wordData;
  status = XNUCLEO53L1A1_ResetId(ToFNumber, VL53L1X_POWER_OFF); // Reset ToF sensor
  HAL_Delay(2);
  status = XNUCLEO53L1A1_ResetId(ToFNumber, VL53L1X_POWER_ON); // Reset ToF sensor
  HAL_Delay(2);


/* Those basic I2C read functions can be used to check your own I2C functions */
  status = VL53L1_RdByte(VL53L1_address_default, 0x010F, &byteData);
  sprintf(buffer, "VL53L1X Model_ID: %X \n", byteData);
  HAL_UART_Transmit(&huart2 , buffer, sizeof(buffer) , HAL_MAX_DELAY);

  status = VL53L1_RdByte(VL53L1_address_default, 0x0110, &byteData);
  sprintf(buffer, "VL53L1X Module_Type: %X \n", byteData);
  HAL_UART_Transmit(&huart2 , buffer, sizeof(buffer) , HAL_MAX_DELAY);

  status = VL53L1_RdWord(VL53L1_address_default, 0x010F, &wordData);
  sprintf(buffer, "VL53L1X: %X \n", wordData);
  HAL_UART_Transmit(&huart2 , buffer, sizeof(buffer) , HAL_MAX_DELAY);

  while(sensorState==0){
		status = VL53L1X_BootState(VL53L1_address_default, &sensorState);
	  HAL_Delay(2);
  }
  sprintf(buffer, "VL53L1X Model_ID: %X \n", byteData);
  HAL_UART_Transmit(&huart2 , buffer, sizeof(buffer) , HAL_MAX_DELAY);

    /* This function must to be called to initialize the sensor with the default setting  */
  status = VL53L1X_SensorInit(VL53L1_address_default);

  /* Optional functions to be used to change the main ranging parameters according the application requirements to get the best ranging performances */

  status = VL53L1X_SetDistanceMode(VL53L1_address_default, rangingProfile); /* 1=short, 2=long */

  status = VL53L1X_SetTimingBudgetInMs(VL53L1_address_default, timingBudget); /* in ms possible values [20, 50, 100, 200, 500] */
  status = VL53L1X_SetInterMeasurementInMs(VL53L1_address_default, pollingPeriod); /* in ms, IM must be > = TB */

  status = VL53L1X_SetROI(VL53L1_address_default, ROI_X, ROI_Y); /* minimum ROI 4,4 */
  status = VL53L1X_StartRanging(VL53L1_address_default);   /* This function has to be called to enable the ranging */
  status =  VL53L1X_SetI2CAddress(VL53L1_address_default, newToFAddress);
  if (status == 0){
    HAL_UART_Transmit(&huart2 , "VL53L1X Initialized and Calibrated", sizeof("VL53L1X Initialized and Calibrated") , HAL_MAX_DELAY);
  }
}

void getVL53L1A1(VL53L1_Result* TOF_result){
    int status = 0;
    uint8_t buffer[150];

    while (dataReady == 0){
		  status = VL53L1X_CheckForDataReady(TOF_result->Address, &dataReady);
          HAL_Delay(2);
	}
	dataReady = 0;
	status = VL53L1X_GetRangeStatus(TOF_result->Address, &RangeStatus);
	status = VL53L1X_GetDistance(TOF_result->Address, &Distance);
	status = VL53L1X_GetSignalRate(TOF_result->Address, &SignalRate);
	status = VL53L1X_GetAmbientRate(TOF_result->Address, &AmbientRate);
	status = VL53L1X_GetSpadNb(TOF_result->Address, &SpadNum);
	status = VL53L1X_ClearInterrupt(TOF_result->Address); /* clear interrupt has to be called to enable next interrupt*/
    
  TOF_result->Distance = Distance;
  TOF_result->Status = RangeStatus;
  TOF_result->Ambient = AmbientRate;
  TOF_result->Signal = SignalRate;

    // sprintf(buffer, "Distance: %d \n", TOF_result.Distance);
}

__weak bytesToFloat(uint8_t byte1 , uint8_t byte2 , uint8_t byte3 , uint8_t byte4) {
    float result;
    uint8_t bytes[4] = {byte1,byte2,byte3,byte4};

    // Use memcpy to copy the 4 bytes into a float (this preserves the binary representation)
    memcpy(&result, bytes, sizeof(float));

    return result;
}

__weak int32_t bytesToInt32(uint8_t byte1 , uint8_t byte2 , uint8_t byte3 , uint8_t byte4)  {
    int32_t result;
    uint8_t bytes[4] = {byte1,byte2,byte3,byte4};

    // Use memcpy to copy the 4 bytes into a int32_t (this preserves the binary representation)
    memcpy(&result, bytes, sizeof(int32_t));

    return result;
}
