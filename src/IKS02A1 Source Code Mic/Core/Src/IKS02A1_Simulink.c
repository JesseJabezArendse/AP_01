///////////////////////////////////////////////////////////
// Project : STM32 Nucleo + IKS02A1
// Abbreviation : AP_01
// Microcontroller: STM32 Nucleo F411RE
// Made For : Dr. Amir Patel
// @author: Jesse Jabez Arendse
// @modified: 25/11/2024
///////////////////////////////////////////////////////////


// Includes ///////////////////////////////////////////////
#include "main.h"

#include "stdio.h"
#include <stdint.h>
#include <string.h>  // For memcpy
#include <stdio.h>

#include "IKS02A1_Simulink.h"

// Project Parent Variables ///////////////////////////////
const uint8_t* header = 'A_J';
const uint8_t* terminator = 'J_A';

// Project Specific Variables /////////////////////////////

extern UART_HandleTypeDef huart2;


extern uint16_t pdm_buffer;
extern int16_t  pcm_buffer;

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

void sendToSimulink(){
    HAL_UART_Transmit(&huart2, (uint8_t *) &header           ,3 , HAL_MAX_DELAY);
    HAL_UART_Transmit(&huart2, &pcm_buffer       , PCM_BUFFER_SIZE*2 , HAL_MAX_DELAY);
    HAL_UART_Transmit(&huart2, (uint8_t *) &terminator       ,3 , HAL_MAX_DELAY);
}

void startAudioPacket(){
    HAL_UART_Transmit(&huart2, (uint8_t *) &header           ,3 , HAL_MAX_DELAY);
}

void sendAudioChunk(){
    HAL_UART_Transmit_DMA(&huart2, &pcm_buffer, PDM_BUFFER_SIZE*2);
}

void endAudioPacket(){
    HAL_UART_Transmit(&huart2, (uint8_t *) &terminator       ,3 , HAL_MAX_DELAY);
}



void sendEachToSimulink(int16_t val){
    // HAL_UART_Transmit(&huart2, (uint8_t *) &header           ,3 , HAL_MAX_DELAY);
    HAL_UART_Transmit(&huart2, &val       ,2 , HAL_MAX_DELAY);
    // HAL_UART_Transmit(&huart2, (uint8_t *) &terminator       ,3 , HAL_MAX_DELAY);
}