/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "iks02a1_motion_sensors.h"
#include "IKS02A1_Simulink.h"
#include "VL53L8A1_Simulink.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

IKS02A1_MOTION_SENSOR_Axes_t accel1_axis;
IKS02A1_MOTION_SENSOR_Axes_t gyro_axis;
IKS02A1_MOTION_SENSOR_Axes_t accel2_axis;
IKS02A1_MOTION_SENSOR_Axes_t mag_axis;

RANGING_SENSOR_Result_t TOF_centre;
extern RANGING_SENSOR_ProfileConfig_t Profile;

const uint8_t* header = 'A_J';
const uint8_t* terminator = 'J_A';

const uint8_t expectedHeader[3] = {'J', '_', 'A'};
const uint8_t expectedTerminator[3] = {'A', '_', 'J'};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_TIM2_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
const int numberOfSimulinkBytes = 4*8;
uint8_t bigBuffer[4*8 +3+3] = {0};
uint8_t clearToSend = 0;
uint8_t clearToGetL8 = 0;
uint8_t calibrated = 0;
int32_t counter = 0;

extern int32_t accel1_fsr;
extern float accel1_odr;
extern int32_t gyro_fsr;
extern float gyro_odr;
extern int32_t accel2_fsr;
extern float accel2_odr;
extern float mag_odr;
extern float_t temperature;

extern float tof_odr;

extern float fastestODR;

float   bytesToFloat_main(uint8_t byte1 , uint8_t byte2 , uint8_t byte3 , uint8_t byte4) {
    float result;
    uint8_t bytes[4] = {byte1,byte2,byte3,byte4};

    // Use memcpy to copy the 4 bytes into a float (this preserves the binary representation)
    memcpy(&result, bytes, sizeof(float));

    return result;
}

int32_t bytesToInt32_main(uint8_t byte1 , uint8_t byte2 , uint8_t byte3 , uint8_t byte4)  {
    int32_t result;
    uint8_t bytes[4] = {byte1,byte2,byte3,byte4};

    // Use memcpy to copy the 4 bytes into a int32_t (this preserves the binary representation)
    memcpy(&result, bytes, sizeof(int32_t));

    return result;
}

void receivedFromSimulink(uint8_t* bigBuffer){
    accel1_fsr = bytesToInt32_main(bigBuffer[3 + 0] ,  bigBuffer[3 + 1] ,  bigBuffer[3 + 2] ,   bigBuffer[3 + 3] );
    accel1_odr = bytesToFloat_main(bigBuffer[7 + 0] ,  bigBuffer[7 + 1] ,  bigBuffer[7 + 2]  ,  bigBuffer[7 + 3] );
    gyro_fsr =   bytesToInt32_main(bigBuffer[11 + 0] , bigBuffer[11 + 1] , bigBuffer[11 + 2] ,  bigBuffer[11 + 3] );
    gyro_odr =   bytesToFloat_main(bigBuffer[15 + 0] , bigBuffer[15 + 1] , bigBuffer[15 + 2] ,  bigBuffer[15 + 3] );
    accel2_fsr = bytesToInt32_main(bigBuffer[19 + 0] , bigBuffer[19 + 1] , bigBuffer[19 + 2] ,  bigBuffer[19 + 3] );
    accel2_odr = bytesToFloat_main(bigBuffer[23 + 0] , bigBuffer[23 + 1] , bigBuffer[23 + 2] ,  bigBuffer[23 + 3] );
    mag_odr =    bytesToFloat_main(bigBuffer[27 + 0] , bigBuffer[27 + 1] , bigBuffer[27 + 2] ,  bigBuffer[27 + 3] );

    tof_odr =    bytesToFloat_main(bigBuffer[31 + 0] , bigBuffer[31 + 1] , bigBuffer[31 + 2] ,  bigBuffer[31 + 3] );
}

void configureTimer(float desired_frequency, TIM_TypeDef* tim) {
    // Assuming the clock frequency driving the timer is 100 MHz
    float clock_frequency = SystemCoreClock; // 100 MHz

    // Calculate the required total timer period in timer clock cycles
    float timer_period = clock_frequency / desired_frequency;

    // Choose a suitable prescaler (PSC) to fit the period within ARR's range
    uint32_t prescaler = (uint32_t)(timer_period / 65536.0f); // PSC ensures ARR <= 65535
    if (prescaler > 65535) {
        prescaler = 65535; // Cap PSC if it exceeds 16-bit value
    }

    // Calculate the ARR based on the chosen PSC
    uint64_t arr = (uint64_t)(timer_period / (prescaler + 1));



    // Update the timer registers
    tim->PSC = prescaler;   // Set the prescaler
    tim->ARR = arr;         // Set the auto-reload register

    // Reload the timer settings to apply the changes immediately
    tim->EGR = TIM_EGR_UG;  // Generate an update event to reload PSC and ARR
}
// Function to return the fastest (highest) ODR
float get_fastest_odr(float odr1, float odr2, float odr3, float odr4, float odr5) {
    float fastest = odr1; // Assume odr1 is the fastest initially

    if (odr2 > fastest) {
        fastest = odr2;
    }
    if (odr3 > fastest) {
        fastest = odr3;
    }
    if (odr4 > fastest) {
        fastest = odr4;
    }
    if (odr5 > fastest) {
        fastest = odr5;
    }

    return fastest;
}

void sendToSimulink(){
    HAL_UART_Transmit(&huart2, (uint8_t *) &header           ,3 , HAL_MAX_DELAY);

    HAL_UART_Transmit(&huart2, (int32_t *) &(accel1_axis.x)  ,4 , HAL_MAX_DELAY);
    HAL_UART_Transmit(&huart2, (int32_t *) &accel1_axis.y    ,4 , HAL_MAX_DELAY);
    HAL_UART_Transmit(&huart2, (int32_t *) &accel1_axis.z    ,4 , HAL_MAX_DELAY);
    HAL_UART_Transmit(&huart2, (int32_t *) &gyro_axis.x      ,4 , HAL_MAX_DELAY);
    HAL_UART_Transmit(&huart2, (int32_t *) &gyro_axis.y      ,4 , HAL_MAX_DELAY);
    HAL_UART_Transmit(&huart2, (int32_t *) &gyro_axis.z      ,4 , HAL_MAX_DELAY);
    HAL_UART_Transmit(&huart2, (int32_t *) &accel2_axis.x    ,4 , HAL_MAX_DELAY);
    HAL_UART_Transmit(&huart2, (int32_t *) &accel2_axis.y    ,4 , HAL_MAX_DELAY);
    HAL_UART_Transmit(&huart2, (int32_t *) &accel2_axis.z    ,4 , HAL_MAX_DELAY);
    HAL_UART_Transmit(&huart2, (float_t *) &temperature      ,4 , HAL_MAX_DELAY);
    HAL_UART_Transmit(&huart2, (int32_t *) &mag_axis.x       ,4 , HAL_MAX_DELAY);
    HAL_UART_Transmit(&huart2, (int32_t *) &mag_axis.y       ,4 , HAL_MAX_DELAY);
    HAL_UART_Transmit(&huart2, (int32_t *) &mag_axis.z       ,4 , HAL_MAX_DELAY);

    for (int i = 0 ; i < 8*8 ; i++){
        HAL_UART_Transmit(&huart2, (uint32_t *) &((TOF_centre .ZoneResult[i]) .Distance  [0])  , 4 , HAL_MAX_DELAY);
        HAL_UART_Transmit(&huart2, (float_t *)  &((TOF_centre .ZoneResult[i]) .Ambient   [0])  , 4 , HAL_MAX_DELAY);
        HAL_UART_Transmit(&huart2, (float_t *)  &((TOF_centre .ZoneResult[i]) .Signal    [0])  , 4 , HAL_MAX_DELAY);
        HAL_UART_Transmit(&huart2, (float_t *)  &((TOF_centre .ZoneResult[i]) .Status    [0])  , 4 , HAL_MAX_DELAY);
    }

    HAL_UART_Transmit(&huart2, (uint32_t *) &counter          ,4 , HAL_MAX_DELAY);

    HAL_UART_Transmit(&huart2, (uint8_t *) &terminator       ,3 , HAL_MAX_DELAY);
}

void initialCalibration(){
  HAL_UART_Receive(&huart2, &bigBuffer, 4*8+3+3 ,HAL_MAX_DELAY);
  if (bigBuffer[0] == expectedHeader[0] &&
      bigBuffer[1] == expectedHeader[1] &&
      bigBuffer[2] == expectedHeader[2]){
        calibrated = 1;
        HAL_GPIO_WritePin(GREEN_LED_GPIO_Port , GREEN_LED_Pin , 1);
        receivedFromSimulink(&bigBuffer);
        fastestODR = get_fastest_odr(accel1_odr,gyro_odr,accel2_odr,mag_odr,tof_odr);
  }
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM2_Init();
  MX_USART2_UART_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  while (calibrated != 1){
    initialCalibration();
  }
  calibrateVL53L8A1();
  initVL53L8A1();

  initIKS02A1();
  calibrate_IKS02A1();

  configureTimer(fastestODR,TIM2);
  configureTimer(tof_odr,TIM3);
  HAL_TIM_Base_Start_IT(&htim2);
  HAL_TIM_Base_Start_IT(&htim3);
  HAL_GPIO_WritePin(GREEN_LED_GPIO_Port,GREEN_LED_Pin,1);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    getIKS02A1();
    if (clearToGetL8 == 1){
      getVL53L8A1();
      clearToGetL8 = 0;
    }

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 100;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 1000-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1000-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 1843200;
  huart2.Init.WordLength = UART_WORDLENGTH_9B;
  huart2.Init.StopBits = UART_STOPBITS_2;
  huart2.Init.Parity = UART_PARITY_EVEN;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GREEN_LED_GPIO_Port, GREEN_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6|VL53L8A1_PWR_EN_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, VL53L8A1_LOW_PWR_Pin|GPIO_PIN_10|GPIO_PIN_4, GPIO_PIN_SET);

  /*Configure GPIO pin : BLUE_BUTTON_Pin */
  GPIO_InitStruct.Pin = BLUE_BUTTON_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BLUE_BUTTON_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PC0 PC1 PC2 PC3
                           PC4 PC5 PC6 PC7
                           PC8 PC9 PC10 PC11
                           PC12 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7
                          |GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11
                          |GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA0 PA1 PA8 PA9
                           PA10 PA11 PA12 PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_8|GPIO_PIN_9
                          |GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : VL53L8A1_INT_Pin */
  GPIO_InitStruct.Pin = VL53L8A1_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(VL53L8A1_INT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : GREEN_LED_Pin */
  GPIO_InitStruct.Pin = GREEN_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GREEN_LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PA6 VL53L8A1_PWR_EN_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_6|VL53L8A1_PWR_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : VL53L8A1_LOW_PWR_Pin PB10 PB4 */
  GPIO_InitStruct.Pin = VL53L8A1_LOW_PWR_Pin|GPIO_PIN_10|GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB1 PB2 PB12 PB13
                           PB14 PB15 PB3 PB5
                           PB6 PB7 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_12|GPIO_PIN_13
                          |GPIO_PIN_14|GPIO_PIN_15|GPIO_PIN_3|GPIO_PIN_5
                          |GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PD2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
