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
#include "VL53L1A1_Simulink.h"
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
DMA_HandleTypeDef hdma_usart2_tx;

/* USER CODE BEGIN PV */
RANGING_SENSOR_Result_t TOF_left;
RANGING_SENSOR_Result_t TOF_centre;
RANGING_SENSOR_Result_t TOF_right;
RANGING_SENSOR_ProfileConfig_t Profile;


IKS02A1_MOTION_SENSOR_Axes_t accel1_axis;
IKS02A1_MOTION_SENSOR_Axes_t gyro_axis;
IKS02A1_MOTION_SENSOR_Axes_t accel2_axis;
IKS02A1_MOTION_SENSOR_Axes_t mag_axis;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
const uint8_t* header = 'A_J';
const uint8_t* terminator = 'J_A';

const uint8_t expectedHeader[3] = {'J', '_', 'A'};
const uint8_t expectedTerminator[3] = {'A', '_', 'J'};
const int numberOfSimulinkBytes = 4*(2+2+2+2+1);
uint8_t bigBuffer[36+3+3] = {0};
uint8_t clearToSend = 0;
uint8_t clearToGetL1 = 0;
uint8_t calibrated = 0;
int32_t counter = 0;

extern int32_t tof_fsr;
extern float tof_odr;

extern int32_t accel1_fsr;
extern float accel1_odr;
extern int32_t gyro_fsr;
extern float gyro_odr;
extern int32_t accel2_fsr;
extern float accel2_odr;
extern float mag_odr;
extern float_t temperature;

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
    tof_fsr = bytesToInt32_main(bigBuffer[3  + 0] , bigBuffer[3  + 1] , bigBuffer[3  + 2] ,  bigBuffer[3  + 3] );
    tof_odr = bytesToFloat_main(bigBuffer[7  + 0] , bigBuffer[7  + 1] , bigBuffer[7  + 2] ,  bigBuffer[7  + 3] );
    

    accel1_fsr = bytesToInt32_main(bigBuffer[11 + 0] , bigBuffer[11 + 1] , bigBuffer[11 + 2] ,  bigBuffer[11 + 3] );
    accel1_odr = bytesToFloat_main(bigBuffer[15 + 0] , bigBuffer[15 + 1] , bigBuffer[15 + 2] ,  bigBuffer[15 + 3] );
    gyro_fsr =   bytesToInt32_main(bigBuffer[19 + 0] , bigBuffer[19 + 1] , bigBuffer[19 + 2] ,  bigBuffer[19 + 3] );
    gyro_odr =   bytesToFloat_main(bigBuffer[23 + 0] , bigBuffer[23 + 1] , bigBuffer[23 + 2] ,  bigBuffer[23 + 3] );
    accel2_fsr = bytesToInt32_main(bigBuffer[27 + 0] , bigBuffer[27 + 1] , bigBuffer[27 + 2] ,  bigBuffer[27 + 3] );
    accel2_odr = bytesToFloat_main(bigBuffer[31 + 0] , bigBuffer[31 + 1] , bigBuffer[31 + 2] ,  bigBuffer[31 + 3] );
    mag_odr =    bytesToFloat_main(bigBuffer[35 + 0] , bigBuffer[35 + 1] , bigBuffer[35 + 2] ,  bigBuffer[35 + 3] );
    
}

void configureTimer(float desired_frequency) {
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
    uint32_t arr = (uint32_t)(timer_period / (prescaler + 1));
    if (arr > 65535) {
        arr = 65535; // Cap ARR if it exceeds 16-bit value
    }

    // Update the timer registers
    TIM2->PSC = prescaler;   // Set the prescaler
    TIM2->ARR = arr;         // Set the auto-reload register

    // Reload the timer settings to apply the changes immediately
    TIM2->EGR = TIM_EGR_UG;  // Generate an update event to reload PSC and ARR
}

void configureOtherTimer(float desired_frequency) {
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
    uint32_t arr = (uint32_t)(timer_period / (prescaler + 1));
    if (arr > 65535) {
        arr = 65535; // Cap ARR if it exceeds 16-bit value
    }

    // Update the timer registers
    TIM3->PSC = prescaler;   // Set the prescaler
    TIM3->ARR = arr;         // Set the auto-reload register

    // Reload the timer settings to apply the changes immediately
    TIM3->EGR = TIM_EGR_UG;  // Generate an update event to reload PSC and ARR
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

    HAL_UART_Transmit(&huart2, (uint32_t *) &((TOF_left   .ZoneResult[0]) .Distance  [0])  , 4 , HAL_MAX_DELAY);
    HAL_UART_Transmit(&huart2, (float_t *)  &((TOF_left   .ZoneResult[0]) .Ambient   [0])  , 4 , HAL_MAX_DELAY);
    HAL_UART_Transmit(&huart2, (float_t *)  &((TOF_left   .ZoneResult[0]) .Signal    [0])  , 4 , HAL_MAX_DELAY);
    HAL_UART_Transmit(&huart2, (uint32_t *) &((TOF_centre .ZoneResult[0]) .Distance  [0])  , 4 , HAL_MAX_DELAY);
    HAL_UART_Transmit(&huart2, (float_t *)  &((TOF_centre .ZoneResult[0]) .Ambient   [0])  , 4 , HAL_MAX_DELAY);
    HAL_UART_Transmit(&huart2, (float_t *)  &((TOF_centre .ZoneResult[0]) .Signal    [0])  , 4 , HAL_MAX_DELAY);
    HAL_UART_Transmit(&huart2, (uint32_t *) &((TOF_right  .ZoneResult[0]) .Distance  [0])  , 4 , HAL_MAX_DELAY);
    HAL_UART_Transmit(&huart2, (float_t *)  &((TOF_right  .ZoneResult[0]) .Ambient   [0])  , 4 , HAL_MAX_DELAY);
    HAL_UART_Transmit(&huart2, (float_t *)  &((TOF_right  .ZoneResult[0]) .Signal    [0])  , 4 , HAL_MAX_DELAY);

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
    HAL_UART_Transmit(&huart2, (int32_t *) &counter          ,4 , HAL_MAX_DELAY);
    HAL_UART_Transmit(&huart2, (float_t *) &fastestODR       ,4 , HAL_MAX_DELAY);

    HAL_UART_Transmit(&huart2, (uint8_t *) &terminator       ,3 , HAL_MAX_DELAY);
}

void initialCalibration(){
  HAL_UART_Receive(&huart2,(uint8_t *) &bigBuffer, (size_t) (numberOfSimulinkBytes+3+3),1);
  if (bigBuffer[0] == expectedHeader[0] &&
      bigBuffer[1] == expectedHeader[1] &&
      bigBuffer[2] == expectedHeader[2] &&
      bigBuffer[numberOfSimulinkBytes+3+0] == expectedTerminator[0] &&
      bigBuffer[numberOfSimulinkBytes+3+1] == expectedTerminator[1] &&
      bigBuffer[numberOfSimulinkBytes+3+2] == expectedTerminator[2]){
        calibrated = 1;
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
  HAL_GPIO_WritePin(LD2_GPIO_Port,LD2_Pin,0);
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  while (calibrated != 1){
    initialCalibration();
  }
  calibrate_VL53L1A1();
  initVL53L1A1();
  initIKS02A1();
  calibrate_IKS02A1();

  configureOtherTimer(tof_odr);
  configureTimer(fastestODR);
  HAL_TIM_Base_Start_IT(&htim2);
  HAL_TIM_Base_Start_IT(&htim3);
  HAL_GPIO_WritePin(LD2_GPIO_Port,LD2_Pin,1);


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    getIKS02A1(); 
    // (void)VL53L1_GetMeasurementDataReady(pObj, &NewDataReady);
    
    if (clearToGetL1 == 1){
      getVL53L1A1();
      clearToGetL1 = 0;
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
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
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
  /* DMA1_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);
  /* DMA1_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : L1_INT_Pin */
  GPIO_InitStruct.Pin = L1_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(L1_INT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

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
