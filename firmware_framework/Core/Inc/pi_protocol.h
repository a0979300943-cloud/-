/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    pi_protocol.h
  * @brief   树莓派通信协议头文件
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef PI_PROTOCOL_H
#define PI_PROTOCOL_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"

/* 协议常量 */
#define PI_SOF_BYTE1            0xA5U
#define PI_SOF_BYTE2            0x5AU
#define PI_PROTOCOL_VERSION     1U
#define PI_MAX_PAYLOAD          128U
#define PI_FRAME_HEADER_SIZE    6U
#define PI_FRAME_CRC_SIZE       2U
#define PI_FRAME_MAX_SIZE       (2U + PI_FRAME_HEADER_SIZE + PI_MAX_PAYLOAD + PI_FRAME_CRC_SIZE)

/* 消息类型 */
typedef enum
{
    PI_MSG_HELLO            = 0x01,
    PI_MSG_HEARTBEAT        = 0x02,
    PI_MSG_DETECTION        = 0x10,
    PI_MSG_FRAME_DONE       = 0x11,
    PI_MSG_MOTOR_CMD        = 0x20,
    PI_MSG_ENCODER_TELEM    = 0x30,
    PI_MSG_DISTANCE_TELEM   = 0x31,
    PI_MSG_ACK              = 0x80,
    PI_MSG_STATUS           = 0x81
} PiMsgType_t;

/* 检测目标结构体 */
typedef struct
{
    uint8_t  class_id;          /* 目标类别: 0=普通物资, 1=核心物资, 2=伤员, 3=危险目标 */
    uint16_t confidence;        /* 置信度 0-1000 (千分比) */
    int16_t  center_x;          /* 中心X坐标 -1000~1000 (归一化) */
    int16_t  center_y;          /* 中心Y坐标 -1000~1000 (归一化) */
    uint16_t width;             /* 宽度 0-1000 (归一化) */
    uint16_t height;            /* 高度 0-1000 (归一化) */
    uint16_t distance_mm;       /* 距离毫米, 0xFFFF=无效 */
    uint16_t track_id;          /* 跟踪ID */
    uint32_t timestamp_ms;      /* 接收时间戳 */
} PiDetection_t;

/* 电机命令结构体 */
typedef struct
{
    int16_t  motor_1;           /* 电机1速度 -1000~1000 */
    int16_t  motor_2;           /* 电机2速度 -1000~1000 */
    int16_t  motor_3;           /* 电机3速度 -1000~1000 */
    uint16_t ttl_ms;            /* 生存时间 (毫秒) */
} PiMotorCmd_t;

/* 协议解析状态机状态 */
typedef enum
{
    PI_PARSE_IDLE = 0,          /* 等待帧头 */
    PI_PARSE_SOF2,              /* 已收帧头1，等待帧头2 */
    PI_PARSE_HEADER,            /* 接收帧头 */
    PI_PARSE_PAYLOAD,           /* 接收负载 */
    PI_PARSE_CRC1,              /* 接收CRC低字节 */
    PI_PARSE_CRC2               /* 接收CRC高字节 */
} PiParseState_t;

/* 协议上下文 */
typedef struct
{
    UART_HandleTypeDef *huart;
    uint8_t             rx_byte;
    uint8_t             frame_buf[PI_FRAME_MAX_SIZE];
    uint16_t            frame_len;
    uint16_t            frame_pos;
    PiParseState_t      state;

    /* 消息缓存 */
    volatile uint8_t    motor_cmd_pending;
    volatile uint8_t    motor_cmd_valid;
    PiMotorCmd_t        motor_cmd;
    volatile uint32_t   motor_cmd_tick;

    PiDetection_t       latest_detection;
    volatile uint8_t    detection_valid;
    volatile uint32_t   detection_tick;

    volatile uint32_t   heartbeat_tick;
    volatile uint8_t    heartbeat_valid;

    /* 统计 */
    uint16_t            tx_seq;
    uint32_t            crc_errors;
    uint32_t            len_errors;
    uint32_t            rx_frames;
} PiProtocol_t;

/* 接口函数 */
void PiProtocol_Init(PiProtocol_t *proto, UART_HandleTypeDef *huart);
HAL_StatusTypeDef PiProtocol_Start(PiProtocol_t *proto);
void PiProtocol_OnRxComplete(PiProtocol_t *proto, UART_HandleTypeDef *huart);
void PiProtocol_OnUartError(PiProtocol_t *proto, UART_HandleTypeDef *huart);

uint8_t PiProtocol_TakeMotorCmd(PiProtocol_t *proto, PiMotorCmd_t *cmd);
uint8_t PiProtocol_IsMotorCmdExpired(const PiProtocol_t *proto, uint32_t now_ms);

HAL_StatusTypeDef PiProtocol_SendEncoderTelem(PiProtocol_t *proto,
                                               int32_t enc1, int32_t enc2, int32_t enc3);
HAL_StatusTypeDef PiProtocol_SendDistanceTelem(PiProtocol_t *proto,
                                                uint16_t left_mm, uint16_t right_mm,
                                                uint8_t valid_mask);
HAL_StatusTypeDef PiProtocol_SendStatus(PiProtocol_t *proto, uint8_t status_code);

#ifdef __cplusplus
}
#endif

#endif /* PI_PROTOCOL_H */
