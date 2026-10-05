/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    pi_protocol.c
  * @brief   树莓派通信协议实现
  ******************************************************************************
  */
/* USER CODE END Header */

#include "pi_protocol.h"
#include <string.h>

/* 内部函数声明 */
static void PiProtocol_ParseByte(PiProtocol_t *proto, uint8_t byte);
static void PiProtocol_HandleFrame(PiProtocol_t *proto);
static uint16_t PiProtocol_Crc16(const uint8_t *data, uint16_t len);

/**
  * @brief  协议初始化
  */
void PiProtocol_Init(PiProtocol_t *proto, UART_HandleTypeDef *huart)
{
    memset(proto, 0, sizeof(PiProtocol_t));
    proto->huart = huart;
    proto->state = PI_PARSE_IDLE;
    proto->tx_seq = 0U;
}

/**
  * @brief  启动 UART 接收 (中断模式，单字节)
  */
HAL_StatusTypeDef PiProtocol_Start(PiProtocol_t *proto)
{
    return HAL_UART_Receive_IT(proto->huart, &proto->rx_byte, 1U);
}

/**
  * @brief  UART 接收完成回调 (在 HAL_UART_RxCpltCallback 中调用)
  */
void PiProtocol_OnRxComplete(PiProtocol_t *proto, UART_HandleTypeDef *huart)
{
    if (huart == proto->huart)
    {
        PiProtocol_ParseByte(proto, proto->rx_byte);
        (void)HAL_UART_Receive_IT(proto->huart, &proto->rx_byte, 1U);
    }
}

/**
  * @brief  UART 错误回调 (在 HAL_UART_ErrorCallback 中调用)
  */
void PiProtocol_OnUartError(PiProtocol_t *proto, UART_HandleTypeDef *huart)
{
    if (huart == proto->huart)
    {
        /* 清除错误并重启接收 */
        __HAL_UART_CLEAR_FLAG(huart, UART_CLEAR_OREF | UART_CLEAR_NEF |
                                     UART_CLEAR_PEF | UART_CLEAR_FEF);
        (void)HAL_UART_Receive_IT(proto->huart, &proto->rx_byte, 1U);
    }
}

/**
  * @brief  逐字节解析协议帧
  */
static void PiProtocol_ParseByte(PiProtocol_t *proto, uint8_t byte)
{
    switch (proto->state)
    {
        case PI_PARSE_IDLE:
            if (byte == PI_SOF_BYTE1)
            {
                proto->frame_buf[0] = byte;
                proto->frame_pos = 1U;
                proto->state = PI_PARSE_SOF2;
            }
            break;

        case PI_PARSE_SOF2:
            if (byte == PI_SOF_BYTE2)
            {
                proto->frame_buf[proto->frame_pos++] = byte;
                proto->frame_len = 0U;
                proto->state = PI_PARSE_HEADER;
            }
            else
            {
                proto->state = PI_PARSE_IDLE;
                proto->frame_pos = 0U;
            }
            break;

        case PI_PARSE_HEADER:
            proto->frame_buf[proto->frame_pos++] = byte;
            if (proto->frame_pos >= (2U + PI_FRAME_HEADER_SIZE))
            {
                /* 读取负载长度 */
                uint16_t payload_len = (uint16_t)proto->frame_buf[4] |
                                       ((uint16_t)proto->frame_buf[5] << 8);
                if (payload_len > PI_MAX_PAYLOAD)
                {
                    proto->len_errors++;
                    proto->state = PI_PARSE_IDLE;
                    proto->frame_pos = 0U;
                }
                else
                {
                    proto->frame_len = 2U + PI_FRAME_HEADER_SIZE + payload_len;
                    proto->state = PI_PARSE_PAYLOAD;
                }
            }
            break;

        case PI_PARSE_PAYLOAD:
            proto->frame_buf[proto->frame_pos++] = byte;
            if (proto->frame_pos >= proto->frame_len)
            {
                proto->state = PI_PARSE_CRC1;
            }
            break;

        case PI_PARSE_CRC1:
            proto->frame_buf[proto->frame_pos++] = byte;
            proto->state = PI_PARSE_CRC2;
            break;

        case PI_PARSE_CRC2:
            proto->frame_buf[proto->frame_pos++] = byte;
            proto->rx_frames++;
            PiProtocol_HandleFrame(proto);
            proto->state = PI_PARSE_IDLE;
            proto->frame_pos = 0U;
            break;

        default:
            proto->state = PI_PARSE_IDLE;
            proto->frame_pos = 0U;
            break;
    }
}

/**
  * @brief  处理完整帧
  */
static void PiProtocol_HandleFrame(PiProtocol_t *proto)
{
    const uint8_t *body = &proto->frame_buf[2];
    uint8_t version = body[0];
    uint8_t msg_type = body[1];
    uint16_t payload_len = (uint16_t)body[4] | ((uint16_t)body[5] << 8);
    const uint8_t *payload = &body[PI_FRAME_HEADER_SIZE];

    /* CRC 校验 */
    uint16_t recv_crc = (uint16_t)proto->frame_buf[proto->frame_len] |
                        ((uint16_t)proto->frame_buf[proto->frame_len + 1U] << 8);
    uint16_t calc_crc = PiProtocol_Crc16(body, PI_FRAME_HEADER_SIZE + payload_len);

    if (recv_crc != calc_crc)
    {
        proto->crc_errors++;
        return;
    }

    if (version != PI_PROTOCOL_VERSION)
    {
        return;
    }

    uint32_t now = HAL_GetTick();

    switch (msg_type)
    {
        case PI_MSG_HEARTBEAT:
            if (payload_len == 8U)
            {
                proto->heartbeat_tick = now;
                proto->heartbeat_valid = 1U;
            }
            break;

        case PI_MSG_MOTOR_CMD:
            if (payload_len == 8U)
            {
                proto->motor_cmd.motor_1 = (int16_t)(payload[0] | (payload[1] << 8));
                proto->motor_cmd.motor_2 = (int16_t)(payload[2] | (payload[3] << 8));
                proto->motor_cmd.motor_3 = (int16_t)(payload[4] | (payload[5] << 8));
                proto->motor_cmd.ttl_ms  = (uint16_t)(payload[6] | (payload[7] << 8));

                /* TTL 限幅 */
                if (proto->motor_cmd.ttl_ms < 20U)
                    proto->motor_cmd.ttl_ms = 20U;
                else if (proto->motor_cmd.ttl_ms > 2000U)
                    proto->motor_cmd.ttl_ms = 2000U;

                proto->motor_cmd_tick = now;
                proto->motor_cmd_pending = 1U;
                proto->motor_cmd_valid = 1U;
            }
            break;

        case PI_MSG_DETECTION:
            if (payload_len == 16U)
            {
                proto->latest_detection.class_id = payload[0];
                proto->latest_detection.confidence = (uint16_t)(payload[2] | (payload[3] << 8));
                proto->latest_detection.center_x = (int16_t)(payload[4] | (payload[5] << 8));
                proto->latest_detection.center_y = (int16_t)(payload[6] | (payload[7] << 8));
                proto->latest_detection.width = (uint16_t)(payload[8] | (payload[9] << 8));
                proto->latest_detection.height = (uint16_t)(payload[10] | (payload[11] << 8));
                proto->latest_detection.distance_mm = (uint16_t)(payload[12] | (payload[13] << 8));
                proto->latest_detection.track_id = (uint16_t)(payload[14] | (payload[15] << 8));
                proto->latest_detection.timestamp_ms = now;
                proto->detection_valid = 1U;
                proto->detection_tick = now;
            }
            break;

        default:
            break;
    }
}

/**
  * @brief  CRC16-CCITT 计算 (初值 0xFFFF, 多项式 0x1021)
  */
static uint16_t PiProtocol_Crc16(const uint8_t *data, uint16_t len)
{
    uint16_t crc = 0xFFFFU;
    for (uint16_t i = 0U; i < len; i++)
    {
        crc ^= (uint16_t)data[i] << 8;
        for (uint8_t j = 0U; j < 8U; j++)
        {
            if ((crc & 0x8000U) != 0U)
                crc = (uint16_t)((crc << 1) ^ 0x1021U);
            else
                crc = (uint16_t)(crc << 1);
        }
    }
    return crc;
}

/**
  * @brief  取出电机命令
  * @return 1=有新命令, 0=无新命令
  */
uint8_t PiProtocol_TakeMotorCmd(PiProtocol_t *proto, PiMotorCmd_t *cmd)
{
    if (proto->motor_cmd_pending != 0U)
    {
        memcpy(cmd, &proto->motor_cmd, sizeof(PiMotorCmd_t));
        proto->motor_cmd_pending = 0U;
        return 1U;
    }
    return 0U;
}

/**
  * @brief  检查电机命令是否过期
  * @return 1=已过期, 0=有效
  */
uint8_t PiProtocol_IsMotorCmdExpired(const PiProtocol_t *proto, uint32_t now_ms)
{
    if (proto->motor_cmd_valid == 0U)
        return 1U;
    return ((now_ms - proto->motor_cmd_tick) > proto->motor_cmd.ttl_ms) ? 1U : 0U;
}

/**
  * @brief  发送帧到树莓派
  */
static HAL_StatusTypeDef PiProtocol_SendFrame(PiProtocol_t *proto, uint8_t msg_type,
                                               const uint8_t *payload, uint16_t payload_len)
{
    uint8_t frame[PI_FRAME_MAX_SIZE];
    uint16_t pos = 0U;

    frame[pos++] = PI_SOF_BYTE1;
    frame[pos++] = PI_SOF_BYTE2;
    frame[pos++] = PI_PROTOCOL_VERSION;
    frame[pos++] = msg_type;
    frame[pos++] = (uint8_t)(proto->tx_seq & 0xFFU);
    frame[pos++] = (uint8_t)((proto->tx_seq >> 8) & 0xFFU);
    proto->tx_seq++;
    frame[pos++] = (uint8_t)(payload_len & 0xFFU);
    frame[pos++] = (uint8_t)((payload_len >> 8) & 0xFFU);

    if ((payload != NULL) && (payload_len > 0U))
    {
        memcpy(&frame[pos], payload, payload_len);
        pos = (uint16_t)(pos + payload_len);
    }

    uint16_t crc = PiProtocol_Crc16(&frame[2], (uint16_t)(pos - 2U));
    frame[pos++] = (uint8_t)(crc & 0xFFU);
    frame[pos++] = (uint8_t)((crc >> 8) & 0xFFU);

    return HAL_UART_Transmit(proto->huart, frame, pos, 100U);
}

/**
  * @brief  发送编码器遥测
  */
HAL_StatusTypeDef PiProtocol_SendEncoderTelem(PiProtocol_t *proto,
                                               int32_t enc1, int32_t enc2, int32_t enc3)
{
    uint8_t payload[12];
    payload[0] = (uint8_t)(enc1 & 0xFFU);
    payload[1] = (uint8_t)((enc1 >> 8) & 0xFFU);
    payload[2] = (uint8_t)((enc1 >> 16) & 0xFFU);
    payload[3] = (uint8_t)((enc1 >> 24) & 0xFFU);
    payload[4] = (uint8_t)(enc2 & 0xFFU);
    payload[5] = (uint8_t)((enc2 >> 8) & 0xFFU);
    payload[6] = (uint8_t)((enc2 >> 16) & 0xFFU);
    payload[7] = (uint8_t)((enc2 >> 24) & 0xFFU);
    payload[8] = (uint8_t)(enc3 & 0xFFU);
    payload[9] = (uint8_t)((enc3 >> 8) & 0xFFU);
    payload[10] = (uint8_t)((enc3 >> 16) & 0xFFU);
    payload[11] = (uint8_t)((enc3 >> 24) & 0xFFU);

    return PiProtocol_SendFrame(proto, PI_MSG_ENCODER_TELEM, payload, 12U);
}

/**
  * @brief  发送测距遥测
  */
HAL_StatusTypeDef PiProtocol_SendDistanceTelem(PiProtocol_t *proto,
                                                uint16_t left_mm, uint16_t right_mm,
                                                uint8_t valid_mask)
{
    uint8_t payload[5];
    payload[0] = (uint8_t)(left_mm & 0xFFU);
    payload[1] = (uint8_t)((left_mm >> 8) & 0xFFU);
    payload[2] = (uint8_t)(right_mm & 0xFFU);
    payload[3] = (uint8_t)((right_mm >> 8) & 0xFFU);
    payload[4] = valid_mask;

    return PiProtocol_SendFrame(proto, PI_MSG_DISTANCE_TELEM, payload, 5U);
}

/**
  * @brief  发送状态
  */
HAL_StatusTypeDef PiProtocol_SendStatus(PiProtocol_t *proto, uint8_t status_code)
{
    return PiProtocol_SendFrame(proto, PI_MSG_STATUS, &status_code, 1U);
}
