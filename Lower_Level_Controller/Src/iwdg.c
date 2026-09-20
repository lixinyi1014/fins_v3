/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    iwdg.c
  * @brief   This file provides code for the configuration
  *          of the IWDG instances.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "iwdg.h"
#include "ControllerRtosHooks.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

IWDG_HandleTypeDef hiwdg;

/* IWDG init function */
void MX_IWDG_Init(void)
{

  /* USER CODE BEGIN IWDG_Init 0 */

  /* USER CODE END IWDG_Init 0 */

  /* USER CODE BEGIN IWDG_Init 1 */

  /* USER CODE END IWDG_Init 1 */
  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_16;
  hiwdg.Init.Reload = 999; // 16*(999+1)/32 kHz ≈ 500 ms，实际时间随 LSI 频差变化。
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN IWDG_Init 2 */

  /* USER CODE END IWDG_Init 2 */

}

/* USER CODE BEGIN 1 */

int LcSafetyHardware_WatchdogReset(void)
{
  int watchdog_reset = __HAL_RCC_GET_FLAG(RCC_FLAG_IWDGRST) != RESET;
  __HAL_RCC_CLEAR_RESET_FLAGS();
#if LC_PCA_OE_ENABLED
  __HAL_RCC_GPIOA_CLK_ENABLE(); __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE(); __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE(); __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE(); __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOI_CLK_ENABLE();
  HAL_GPIO_WritePin(LC_PCA_OE_GPIO_PORT, LC_PCA_OE_GPIO_PIN, GPIO_PIN_SET);
  GPIO_InitTypeDef pin = {0};
  pin.Pin = LC_PCA_OE_GPIO_PIN; pin.Mode = GPIO_MODE_OUTPUT_PP;
  pin.Pull = GPIO_PULLUP; pin.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LC_PCA_OE_GPIO_PORT, &pin);
#endif
  return watchdog_reset;
}
void LcSafetyHardware_SetOutputEnabled(int enabled)
{
#if LC_PCA_OE_ENABLED
  HAL_GPIO_WritePin(LC_PCA_OE_GPIO_PORT, LC_PCA_OE_GPIO_PIN,
                   enabled ? GPIO_PIN_RESET : GPIO_PIN_SET);
#else
  (void)enabled; // TODO(HARDWARE)：当前只靠总线写中位；独立急停应切断推进器动力。
#endif
}
void LcSafetyHardware_Start(void)
{
#if LC_IWDG_ENABLED
  __HAL_DBGMCU_UNFREEZE_IWDG(); // 断点不掩盖故障；带动力时不能依靠暂停 CPU 停机。
  MX_IWDG_Init();
#endif
}
void LcSafetyHardware_Feed(void)
{
#if LC_IWDG_ENABLED
  HAL_IWDG_Refresh(&hiwdg); // 仅完整控制循环结束后喂狗。
#endif
}

/* USER CODE END 1 */
