/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    wdg.c
  * @brief   独立看门狗 (IWDG) 实现
  ******************************************************************************
  */
/* USER CODE END Header */

#include "wdg.h"

static IWDG_HandleTypeDef g_hiwdg;

/**
  * @brief  初始化并启动 IWDG (约 4 秒超时)
  * @note   需在 stm32f4xx_hal_conf.h 中启用 HAL_IWDG_MODULE_ENABLED
  *         启动后必须周期性调用 WDG_Refresh，否则芯片自动复位
  */
void WDG_Init(void)
{
    g_hiwdg.Instance = IWDG;
    g_hiwdg.Init.Prescaler = IWDG_PRESCALER_64;
    g_hiwdg.Init.Reload = 2000U;
    (void)HAL_IWDG_Init(&g_hiwdg);
}

/**
  * @brief  喂狗
  */
void WDG_Refresh(void)
{
    (void)HAL_IWDG_Refresh(&g_hiwdg);
}
