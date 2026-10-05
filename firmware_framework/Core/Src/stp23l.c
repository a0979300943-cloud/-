/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stp23l.c
  * @brief   STP23L 激光测距传感器驱动实现
  ******************************************************************************
  */
/* USER CODE END Header */

#include "stp23l.h"
#include <string.h>

/* STP23L 帧头特征: AA AA AA AA 00 02 00 00 B8 00 */
#define STP23L_HEADER_SIZE      10U
#define STP23L_SAMPLE_OFFSET    10U
#define STP23L_SAMPLE_SIZE      15U
#define STP23L_CONF_OFFSET      8U
#define STP23L_MIN_DIST_MM      30U
#define STP23L_MAX_DIST_MM      7500U
#define STP23L_MIN_CONFIDENCE   50U
#define STP23L_MIN_VALID_SAMP   6U

/* 内部函数声明 */
static void Stp23l_ParseBuffer(Stp23l_t *sensor, const uint8_t *data, uint16_t len);
static void Stp23l_ParseFrame(Stp23l_t *sensor);
static uint16_t Stp23l_ReadU16LE(const uint8_t *data);
static void Stp23l_SortU16(uint16_t *values, uint8_t count);

/**
  * @brief  传感器初始化
  */
void Stp23l_Init(Stp23l_t *sensor, UART_HandleTypeDef *huart)
{
    memset(sensor, 0, sizeof(Stp23l_t));
    sensor->huart = huart;
    sensor->distance_mm = STP23L_INVALID_DIST;
    sensor->valid = 0U;
}

/**
  * @brief  启动 DMA 接收 (循环模式)
  */
HAL_StatusTypeDef Stp23l_Start(Stp23l_t *sensor)
{
    return HAL_UART_Receive_DMA(sensor->huart, sensor->dma_buf, STP23L_DMA_BUF_SIZE);
}

/**
  * @brief  DMA 半完成回调
  */
void Stp23l_OnRxHalfComplete(Stp23l_t *sensor, UART_HandleTypeDef *huart)
{
    if (huart == sensor->huart)
    {
        Stp23l_ParseBuffer(sensor, sensor->dma_buf, STP23L_DMA_BUF_SIZE / 2U);
    }
}

/**
  * @brief  DMA 完成回调
  */
void Stp23l_OnRxComplete(Stp23l_t *sensor, UART_HandleTypeDef *huart)
{
    if (huart == sensor->huart)
    {
        Stp23l_ParseBuffer(sensor,
                           &sensor->dma_buf[STP23L_DMA_BUF_SIZE / 2U],
                           STP23L_DMA_BUF_SIZE / 2U);
    }
}

/**
  * @brief  UART 错误回调
  */
void Stp23l_OnUartError(Stp23l_t *sensor, UART_HandleTypeDef *huart)
{
    if (huart == sensor->huart)
    {
        sensor->uart_errors++;
        __HAL_UART_CLEAR_FLAG(huart, UART_CLEAR_OREF | UART_CLEAR_NEF |
                                     UART_CLEAR_PEF | UART_CLEAR_FEF);
        /* 请求重启 DMA */
        sensor->restart_req = 1U;
    }
}

/**
  * @brief  服务函数 (主循环调用，处理重启请求)
  */
void Stp23l_Service(Stp23l_t *sensor)
{
    if (sensor->restart_req != 0U)
    {
        sensor->restart_req = 0U;
        (void)HAL_UART_AbortReceive(sensor->huart);
        (void)HAL_UART_Receive_DMA(sensor->huart, sensor->dma_buf, STP23L_DMA_BUF_SIZE);
    }
}

/**
  * @brief  从 DMA 缓冲区解析数据
  */
static void Stp23l_ParseBuffer(Stp23l_t *sensor, const uint8_t *data, uint16_t len)
{
    for (uint16_t i = 0U; i < len; i++)
    {
        uint8_t byte = data[i];

        /* 帧同步：查找 4 个连续 0xAA */
        if (sensor->frame_pos < 4U)
        {
            if (byte == 0xAAU)
            {
                sensor->frame_buf[sensor->frame_pos++] = byte;
            }
            else
            {
                sensor->frame_pos = 0U;
            }
        }
        else
        {
            sensor->frame_buf[sensor->frame_pos++] = byte;
        }

        /* 完整帧接收完成 */
        if (sensor->frame_pos >= STP23L_FRAME_SIZE)
        {
            Stp23l_ParseFrame(sensor);
            sensor->frame_pos = 0U;
        }
    }
}

/**
  * @brief  解析完整帧
  */
static void Stp23l_ParseFrame(Stp23l_t *sensor)
{
    /* 验证帧头: AA AA AA AA 00 02 00 00 B8 00 */
    static const uint8_t header[STP23L_HEADER_SIZE] =
    {
        0xAAU, 0xAAU, 0xAAU, 0xAAU, 0x00U,
        0x02U, 0x00U, 0x00U, 0xB8U, 0x00U
    };

    if (memcmp(sensor->frame_buf, header, STP23L_HEADER_SIZE) != 0)
    {
        sensor->invalid_count++;
        return;
    }

    sensor->frame_count++;

    /* 提取 12 组采样数据，取有效值中位数 */
    uint16_t valid_dists[STP23L_SAMPLE_COUNT];
    uint16_t conf_sum = 0U;
    uint8_t valid_cnt = 0U;

    for (uint8_t i = 0U; i < STP23L_SAMPLE_COUNT; i++)
    {
        uint16_t offset = STP23L_SAMPLE_OFFSET + (uint16_t)i * STP23L_SAMPLE_SIZE;
        uint16_t dist = Stp23l_ReadU16LE(&sensor->frame_buf[offset]);
        uint8_t conf = sensor->frame_buf[offset + STP23L_CONF_OFFSET];

        if ((dist >= STP23L_MIN_DIST_MM) && (dist <= STP23L_MAX_DIST_MM) &&
            (conf >= STP23L_MIN_CONFIDENCE))
        {
            valid_dists[valid_cnt++] = dist;
            conf_sum = (uint16_t)(conf_sum + conf);
        }
    }

    if (valid_cnt < STP23L_MIN_VALID_SAMP)
    {
        sensor->invalid_count++;
        return;
    }

    /* 排序取中位数 */
    Stp23l_SortU16(valid_dists, valid_cnt);

    if ((valid_cnt & 1U) != 0U)
    {
        sensor->distance_mm = valid_dists[valid_cnt / 2U];
    }
    else
    {
        sensor->distance_mm = (uint16_t)(
            ((uint32_t)valid_dists[(valid_cnt / 2U) - 1U] +
             (uint32_t)valid_dists[valid_cnt / 2U]) / 2U);
    }

    sensor->confidence = (uint8_t)(conf_sum / valid_cnt);
    sensor->last_update_ms = HAL_GetTick();
    sensor->valid = 1U;
}

/**
  * @brief  读取小端 U16
  */
static uint16_t Stp23l_ReadU16LE(const uint8_t *data)
{
    return (uint16_t)data[0] | ((uint16_t)data[1] << 8);
}

/**
  * @brief  插入排序 (小数据量)
  */
static void Stp23l_SortU16(uint16_t *values, uint8_t count)
{
    for (uint8_t i = 1U; i < count; i++)
    {
        uint16_t val = values[i];
        uint8_t pos = i;
        while ((pos > 0U) && (values[pos - 1U] > val))
        {
            values[pos] = values[pos - 1U];
            pos--;
        }
        values[pos] = val;
    }
}

/**
  * @brief  获取距离值
  * @param  now_ms: 当前时间戳
  * @param  max_age_ms: 最大允许数据年龄
  * @param  distance_mm: 输出距离
  * @return 1=有效, 0=无效/过期
  */
uint8_t Stp23l_GetDistance(const Stp23l_t *sensor, uint32_t now_ms,
                            uint32_t max_age_ms, uint16_t *distance_mm)
{
    if (sensor->valid == 0U)
        return 0U;

    if ((now_ms - sensor->last_update_ms) > max_age_ms)
        return 0U;

    *distance_mm = sensor->distance_mm;
    return 1U;
}
