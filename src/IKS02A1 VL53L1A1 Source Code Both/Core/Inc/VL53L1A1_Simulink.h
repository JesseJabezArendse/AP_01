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

#define INSTANCE_TOF_LEFT    0 
#define INSTANCE_TOF_CENTRE  1 
#define INSTANCE_TOF_RIGHT   2 

#define TIMING_BUDGET (30U) /* 16 ms < TimingBudget < 500 ms */
#define POLLING_PERIOD (250U) /* refresh rate for polling mode (ms, shall be consistent with TimingBudget value) */


// Includes ///////////////////////////////////////////////
#include "53l1a2_ranging_sensor.h"
#include "53l1a2_conf.h"
#include "main.h"

#include "stdio.h"
#include <stdint.h>
#include <string.h>  // For memcpy

// Function Prototypes ////////////////////////////////////

void initVL53L1A1(void);

// Gets the distance measurements from VL53L1A1 sensors
void getVL53L1A1(void);

// Receives data from Simulink
void receivedFromSimulink(uint8_t* bigBuffer);

// Sends data to Simulink
void sendToSimulink(void);

// Calibrates the VL53L1A1 sensors
void calibrate_VL53L1A1(void);

#endif // AP_01_H
