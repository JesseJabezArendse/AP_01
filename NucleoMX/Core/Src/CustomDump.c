/*
Custom USB Dump Library for STM32 Nucleo
@author: Jesse Jabez Arendse
@project: AP_1
*/

#include "main.h"
#include "CustomDump.h"

#include "math.h"
#include <string.h>
#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>

extern UART_HandleTypeDef huart2;
extern double IMU_Accel[3];
extern double IMU_Gyro[3];
extern double IMU_Temp;
extern int16_t Magnetometer[3];


void UnInterruptable(void){
    uint32_t irq;
    // Loop through all possible IRQ numbers
    for (irq = 0; irq < 240; irq++)
    {
        // Skip USB FS
        if (irq == OTG_FS_IRQn || irq == RCC_IRQn)
        {
            continue;
        }
        // Disable the interrupt
        HAL_NVIC_DisableIRQ((IRQn_Type)irq);
    }
}

void Interruptable(void){
    uint32_t irq;
    // Loop through all possible IRQ numbers
    for (irq = 0; irq < 240; irq++){
        HAL_NVIC_EnableIRQ((IRQn_Type)irq);
    }
}

void Dump_IKS02A1(void){
    
    // SIMULINK BYTE UNPACKING AND DEMUXING MUST MATCH THIS
        // UnInterruptable();
        HAL_UART_Transmit(&huart2,"JJA",3,HAL_MAX_DELAY); //header

        HAL_UART_Transmit(&huart2, &IMU_Accel    ,3*8,HAL_MAX_DELAY);
        HAL_UART_Transmit(&huart2, &IMU_Gyro     ,3*8,HAL_MAX_DELAY);
        HAL_UART_Transmit(&huart2, &IMU_Temp     ,1*8,HAL_MAX_DELAY);
        HAL_UART_Transmit(&huart2, &Magnetometer,3*2,HAL_MAX_DELAY);

        HAL_UART_Transmit(&huart2,"AP_1",4,HAL_MAX_DELAY);//terminator
        // Interruptable(); 

        HAL_Delay(20);
} 

void Dump_VL53L8(void){
    // SIMULINK BYTE UNPACKING AND DEMUXING MUST MATCH THIS
    // UnInterruptable();
    // HAL_UART_Transmit(&huart2,"JJA",3,HAL_MAX_DELAY); //header

    // HAL_UART_Transmit(&huart2,"AP_1",4,HAL_MAX_DELAY);//terminator
    // Interruptable(); 

    HAL_Delay(20);
} 