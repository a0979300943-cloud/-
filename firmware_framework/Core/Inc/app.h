/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    app.h
  * @brief   智能救援小车 - 业务总装层 (CubeMX 友好)
  *
  *   main.c 只需要:
  *     USER CODE Includes : #include "app.h"
  *     USER CODE 2        : App_Init();
  *     USER CODE 3        : App_Tick();
  *
  *   所有外设句柄 (htim1..htim10 / huart1..3) 由 CubeMX 生成,
  *   app.c 通过 tim.h / usart.h 中的 extern 声明引用, 不重复定义。
  *   引脚宏 (*_Pin / *_GPIO_Port) 由 CubeMX 的 GPIO User Label 生成,
  *   需在 .ioc 中按 CUBEMX_配置与合并指南.md 命名。
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef APP_H
#define APP_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"

/*===================== 电机 / PWM =====================*/
#define MOTOR_COUNT               3U
#define MOTOR_CMD_MAX             1000
#define MOTOR_CMD_MIN             (-1000)

/*===================== 舵机 =====================*/
#define SERVO_LEFT_INDEX          0U
#define SERVO_RIGHT_INDEX         1U
#define SERVO_CAM_INDEX           2U

/* 铲板舵机角度 (角度×100) */
#define SHOVEL_ANGLE_DOWN         4500U   /* 铲入 */
#define SHOVEL_ANGLE_HOLD         7500U   /* 托举转运 */
#define SHOVEL_ANGLE_UP           13500U  /* 抬升释放 */
#define SERVO_ANGLE_INIT          9000U   /* 上电中位 */

/*===================== 调度周期 (ms) =====================*/
#define CONTROL_PERIOD_MS         10U     /* 编码器/里程计/PID */
#define SAFETY_CHECK_PERIOD_MS    50U
#define TELEMETRY_PERIOD_MS       100U

/*===================== 安全参数 =====================*/
#define STP23L_MAX_AGE_MS         300U
#define HEARTBEAT_TIMEOUT_MS      1000U

/* 设为 1: 发车前必须先收到树莓派心跳 (正式比赛建议 1, 单车调试可 0) */
#define APP_REQUIRE_PI_HEARTBEAT  0

/*===================== PID (计数/10ms, 必须实测整定) =====================*/
#define PID_KP                    0.35f
#define PID_KI                    0.02f
#define PID_KD                    0.05f
#define PID_INTEGRAL_MAX          600.0f
#define ENCODER_DELTA_LIMIT       30000

/* 树莓派命令 -1000~1000 → 编码器计数/10ms 满速映射 (需实测) */
#define PI_CMD_1000_COUNTS        2000

/*===================== 按键 =====================*/
#define BUTTON_DEBOUNCE_MS        50U
#define BUTTON_LONGPRESS_MS       1000U

/*===================== 目标检测 =====================*/
#define DETECTION_CONFIDENCE_MIN  600     /* 千分比 */
#define DETECTION_DISTANCE_MAX    800U    /* mm */
#define DETECTION_TIMEOUT_MS      500U

#define TARGET_CLASS_SUPPLY       0U
#define TARGET_CLASS_CORE         1U
#define TARGET_CLASS_CASUALTY     2U
#define TARGET_CLASS_HAZARD       3U

/*===================== 转运时序/距离 =====================*/
#define TRANSPORT_PUSH_TIME_MS    800U
#define TRANSPORT_RELEASE_TIME_MS 500U
#define APPROACH_STOP_DIST_MM     100U
#define APPROACH_MAX_DIST_MM      700.0f
#define TRANSPORT_DIST_MM         900.0f
#define RETURN_DIST_MM           800.0f
#define SEARCH_TIMEOUT_MS         15000U
#define APPROACH_TIMEOUT_MS       5000U
#define TRANSPORT_TIMEOUT_MS      8000U
#define RETURN_TIMEOUT_MS        5000U

/* 本地兜底速度 (无树莓派命令时) */
#define LOCAL_SEARCH_OMEGA        0.6f
#define LOCAL_APPROACH_VY         250.0f
#define LOCAL_TRANSPORT_VY        200.0f
#define LOCAL_RETURN_VY           (-250.0f)
#define LOCAL_AVOID_VY            (-120.0f)

/*===================== 对外接口 =====================*/

/**
  * @brief  业务初始化: 在 main.c 的 USER CODE BEGIN 2 调用一次
  * @note   必须在所有 MX_*Init() 之后调用
  */
void App_Init(void);

/**
  * @brief  业务主循环: 在 main.c 的 USER CODE BEGIN 3 (while(1) 内) 反复调用
  */
void App_Tick(void);

#ifdef __cplusplus
}
#endif

#endif /* APP_H */
