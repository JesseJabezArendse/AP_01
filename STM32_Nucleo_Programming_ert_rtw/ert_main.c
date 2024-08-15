/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: ert_main.c
 *
 * Code generated for Simulink model 'STM32_Nucleo_Programming'.
 *
 * Model version                  : 1.26
 * Simulink Coder version         : 9.9 (R2023a) 19-Nov-2022
 * C/C++ source code generated on : Wed Aug 14 08:27:27 2024
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: ARM Compatible->ARM Cortex
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#include "STM32_Nucleo_Programming.h"
#include "rtwtypes.h"
#include "MW_target_hardware_resources.h"

volatile int IsrOverrun = 0;
static boolean_T OverrunFlag = 0;
void rt_OneStep(void)
{
  /* Check for overrun. Protect OverrunFlag against preemption */
  if (OverrunFlag++) {
    IsrOverrun = 1;
    OverrunFlag--;
    return;
  }

  __enable_irq();
  STM32_Nucleo_Programming_step();

  /* Get model outputs here */
  __disable_irq();
  OverrunFlag--;
}

volatile boolean_T stopRequested;
volatile boolean_T runModel;
int main(void)
{
  float modelBaseRate = 0.01;
  float systemClock = 100;

  /* Initialize variables */
  stopRequested = false;
  runModel = false;

#if defined(MW_MULTI_TASKING_MODE) && (MW_MULTI_TASKING_MODE == 1)

  MW_ASM (" SVC #1");

#endif

  ;
  (void)systemClock;
  HAL_Init();
  SystemCoreClockUpdate();
  rtmSetErrorStatus(STM32_Nucleo_Programming_M, 0);
  STM32_Nucleo_Programming_initialize();
  ARMCM_SysTick_Config(modelBaseRate);
  runModel = rtmGetErrorStatus(STM32_Nucleo_Programming_M) == (NULL);
  __enable_irq();
  while (runModel) {
    stopRequested = !(rtmGetErrorStatus(STM32_Nucleo_Programming_M) == (NULL));
    runModel = !(stopRequested);
  }

  /* Terminate model */
  STM32_Nucleo_Programming_terminate();
  return 0;
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
