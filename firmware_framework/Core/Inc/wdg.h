/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    wdg.h
  * @brief   独立看门狗 (IWDG) 头文件
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef WDG_H
#define WDG_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"

/* 看门狗超时: LSI≈32kHz / 64分频 = 500Hz, 重装2000 -> 4.0秒 */

void WDG_Init(void);
void WDG_Refresh(void);

#ifdef __cplusplus
}
#endif

#endif /* WDG_H */
