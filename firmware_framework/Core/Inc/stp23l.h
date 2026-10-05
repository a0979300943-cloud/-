/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stp23l.h
  * @brief   STP23L 激光测距传感器驱动头文件
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef STP23L_H
#define STP23L_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"

/* STP23L 帧参数 */
#define STP23L_DMA_BUF_SIZE     512U
#define STP23L_FRAME_SIZE       195U
#define STP23L_SAMPLE_COUNT     12U
#define STP23L_INVALID_DIST     0xFFFFU

/* STP23L 数据结构 */
typedef struct
{
    UART_HandleTypeDef *huart;
    uint8_t             dma_buf[STP23L_DMA_BUF_SIZE];
    uint8_t             frame_buf[STP23L_FRAME_SIZE];
    uint16_t            frame_pos;
    volatile uint16_t   distance_mm;
    volatile uint8_t    confidence;
    volatile uint8_t    valid;
    volatile uint8_t    restart_req;
    volatile uint32_t   last_update_ms;
    volatile uint32_t   frame_count;
    volatile uint32_t   invalid_count;
    volatile uint32_t   uart_errors;
} Stp23l_t;

/* 接口函数 */
void Stp23l_Init(Stp23l_t *sensor, UART_HandleTypeDef *huart);
HAL_StatusTypeDef Stp23l_Start(Stp23l_t *sensor);
void Stp23l_Service(Stp23l_t *sensor);
void Stp23l_OnRxHalfComplete(Stp23l_t *sensor, UART_HandleTypeDef *huart);
void Stp23l_OnRxComplete(Stp23l_t *sensor, UART_HandleTypeDef *huart);
void Stp23l_OnUartError(Stp23l_t *sensor, UART_HandleTypeDef *huart);
uint8_t Stp23l_GetDistance(const Stp23l_t *sensor, uint32_t now_ms,
                            uint32_t max_age_ms, uint16_t *distance_mm);

#ifdef __cplusplus
}
#endif

#endif /* STP23L_H */
