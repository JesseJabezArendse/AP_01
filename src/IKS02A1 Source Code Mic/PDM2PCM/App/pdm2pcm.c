/* USER CODE BEGIN Header */
/**
 ******************************************************************************
  * File Name          : pdm2pcm.c
  * Description        : This file provides code for the configuration
  *                      of the pdm2pcm instances.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "pdm2pcm.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* Global variables ---------------------------------------------------------*/
PDM_Filter_Handler_t PDM1_filter_handler;
PDM_Filter_Config_t PDM1_filter_config;

/* USER CODE BEGIN 1 */
extern uint16_t pcm_buffer_len;
/* USER CODE END 1 */

/* PDM2PCM Init function for IKS02A1 */
void MX_PDM2PCM_Init(void)
{
  /* USER CODE BEGIN PDM2PCM_Init 0 */

  /* USER CODE END PDM2PCM_Init 0 */

  /* PDM filter handler configuration */
  PDM1_filter_handler.bit_order = PDM_FILTER_BIT_ORDER_MSB;  // Most significant bit first
  PDM1_filter_handler.endianness = PDM_FILTER_ENDIANNESS_LE; // Big-endian
  PDM1_filter_handler.high_pass_tap = 2122358088;            // High-pass filter coefficient
  PDM1_filter_handler.in_ptr_channels = 1;                  // Number of input PDM channels
  PDM1_filter_handler.out_ptr_channels = 1;                 // Number of output PCM channels

  /* Initialize the PDM filter handler */
  PDM_Filter_Init(&PDM1_filter_handler);

  /* PDM filter configuration */
  PDM1_filter_config.decimation_factor = PDM_FILTER_DEC_FACTOR_128; // Decimation factor
  PDM1_filter_config.output_samples_number = 16;                   // PCM samples per frame
  PDM1_filter_config.mic_gain = -12;                                // Microphone gain (adjustable)

  /* Set PDM filter configuration */
  PDM_Filter_setConfig(&PDM1_filter_handler, &PDM1_filter_config);

  /* USER CODE BEGIN PDM2PCM_Init 1 */

  /* USER CODE END PDM2PCM_Init 1 */
}

/* USER CODE BEGIN 4 */

/*  process function */
uint8_t MX_PDM2PCM_Process(uint16_t *PDMBuf, uint16_t *PCMBuf)
{
  /*
  uint8_t BSP_AUDIO_IN_PDMToPCM(uint16_t * PDMBuf, uint16_t * PCMBuf)

  Converts audio format from PDM to PCM.
  Parameters:
    PDMBuf : Pointer to PDM buffer data
    PCMBuf : Pointer to PCM buffer data
  Return values:
    AUDIO_OK in case of success, AUDIO_ERROR otherwise
  */
  /* this example return the default status AUDIO_ERROR */

  PDM_Filter((uint8_t*) PDMBuf, (uint16_t*) PCMBuf ,&PDM1_filter_handler);

  return (uint8_t) 1;
}

/* USER CODE END 4 */

/**
  * @}
  */
