///////////////////////////////////////////////////////////
// Project : STM32 Nucleo + VL53L1A1
// Abbreviation : AP_01
// Microcontroller: STM32 Nucleo F411RE
// Made For : Dr. Amir Patel
// @author: Jesse Jabez Arendse
// @modified: 24/08/2024
///////////////////////////////////////////////////////////

#ifndef AP_01_H
#define AP_01_H

// Includes ///////////////////////////////////////////////
#include "main.h"

#include "stdio.h"
#include <stdint.h>
#include <string.h>  // For memcpy

#define XNUCLEO53L1A1_DEV_LEFT    0 
#define XNUCLEO53L1A1_DEV_CENTER  1 
#define XNUCLEO53L1A1_DEV_RIGHT   2   
#define RANGING_SENSOR_INSTANCES_NBR 3

#define ToF_Left    0x54 
#define ToF_Centre  0x56
#define ToF_Right   0x58

typedef struct
{
  uint16_t Address;   /*!< I2C Address */
  uint32_t Distance;  /*!< millimeters */
  uint32_t Status;    /*!< OK: 0, NOK: !0 */
  float Ambient;    /*!< kcps / spad */
  float Signal;     /*!< kcps / spad */
} VL53L1_Result;

// Function Prototypes ////////////////////////////////////

void initVL53L1A1(int ToFNumber, uint8_t newToFAddress);

void getVL53L1A1(VL53L1_Result* TOF_result);

#endif // AP_01_H
