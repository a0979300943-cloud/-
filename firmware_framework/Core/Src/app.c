/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    app.c
  * @brief   智能救援小车 - 业务总装层实现
  *
  * 本文件不包含任何 MspInit / SystemClock / MX_*Init,
  * 外设初始化全部由 CubeMX 生成代码完成:
  *   - 电机 PWM  : htim1 (CH1-4), htim8 (CH1-2)
  *   - 编码器     : htim2/htim3/htim4 (Encoder TI12)
  *   - 舵机 PWM  : htim9 (CH1-2), htim10 (CH1)
  *   - 通信       : huart1 (树莓派), huart2/3 (STP23L DMA)
  ******************************************************************************
  */
/* USER CODE END Header */

#include "app.h"

/* CubeMX 生成的头文件: 提供外设句柄 extern 声明 */
#include "main.h"
#include "tim.h"
#include "usart.h"

/* 业务模块 */
#include <math.h>
#include "pi_protocol.h"
#include "stp23l.h"
#include "motor_pid.h"
#include "kinematics.h"
#include "odometry.h"
#include "wdg.h"

/*==================== 内部类型 ====================*/

typedef enum
{
    ROBOT_STATE_IDLE = 0,
    ROBOT_STATE_SEARCH,
    ROBOT_STATE_APPROACH,
    ROBOT_STATE_GRAB,
    ROBOT_STATE_TRANSPORT,
    ROBOT_STATE_RELEASE,
    ROBOT_STATE_RETURN,
    ROBOT_STATE_AVOID,
    ROBOT_STATE_EMERGENCY_STOP,
    ROBOT_STATE_ERROR
} RobotState_t;

typedef enum
{
    CTRL_IDLE = 0,
    CTRL_PI,
    CTRL_INTERNAL
} ControlMode_t;

typedef struct
{
    uint8_t  class_id;
    uint8_t  loaded;
    uint16_t distance_mm;
    int16_t  center_x;
    uint32_t timestamp_ms;
} TransportTarget_t;

typedef struct
{
    uint8_t supply_count;
    uint8_t core_count;
    uint8_t casualty_count;
} TransportStats_t;

/*==================== 内部变量 ====================*/

static RobotState_t  g_robot_state = ROBOT_STATE_IDLE;
static uint32_t     g_state_enter_tick = 0;
static float        g_state_start_x = 0.0f;
static float        g_state_start_y = 0.0f;

static ControlMode_t g_ctrl_mode = CTRL_IDLE;
static int16_t      g_target_counts[3] = {0, 0, 0};
static uint32_t     g_pi_cmd_tick = 0;
static uint16_t     g_pi_cmd_ttl = 0;

static MotorPid_t   g_motor_pid[MOTOR_COUNT];

static int32_t      g_encoder_delta[3] = {0, 0, 0};
static uint32_t     g_encoder_last[3] = {0, 0, 0};
static uint32_t     g_control_tick = 0;

static Odometry_t   g_odom;

static Stp23l_t     g_stp_left;
static Stp23l_t     g_stp_right;
static uint16_t     g_distance_left = STP23L_INVALID_DIST;
static uint16_t     g_distance_right = STP23L_INVALID_DIST;
static uint8_t      g_distance_valid = 0U;

static PiProtocol_t g_pi;
static uint32_t     g_telemetry_tick = 0;

static uint32_t     g_safety_tick = 0;
static volatile uint8_t g_collision_left = 0U;
static volatile uint8_t g_collision_right = 0U;

/* 按键 (EXTI 只置事件, 消抖在主循环) */
static volatile uint8_t g_btn_exti_event = 0U;
static uint8_t      g_btn_raw_level = 1U;
static uint8_t      g_btn_stable_level = 1U;
static uint32_t     g_btn_debounce_tick = 0;
static uint32_t     g_btn_press_tick = 0;
static uint8_t      g_btn_long_fired = 0U;
static volatile uint8_t g_btn_short_press = 0U;
static volatile uint8_t g_btn_long_press = 0U;

static uint8_t      g_first_transport = 0U;
static TransportTarget_t g_target = {0};
static TransportStats_t  g_stats = {0};

static uint32_t     g_led_tick = 0;

/* 调试变量 (ST-LINK Watch 观察, app.h 中 extern) */
volatile int32_t  debug_enc_delta[3] = {0, 0, 0};
volatile int16_t  debug_motor_target[3] = {0, 0, 0};
volatile int16_t  debug_motor_output[3] = {0, 0, 0};
volatile uint16_t debug_distance_left = 0xFFFFU;
volatile uint16_t debug_distance_right = 0xFFFFU;
volatile float    debug_odom_x = 0.0f;
volatile float    debug_odom_y = 0.0f;
volatile float    debug_odom_theta = 0.0f;
volatile uint32_t debug_state = 0;
volatile uint32_t debug_ctrl_mode = 0;

/*==================== 前向声明 ====================*/

static void MotorInit(void);
static void MotorSetRaw(uint8_t motor_id, int16_t command);
static void EncoderInit(void);
static void EncoderUpdate(void);
static void ServoInit(void);
static void ServoSetAngle(uint8_t servo_id, uint16_t angle_x100);

static void DriveChassis(float vx_mm_s, float vy_mm_s, float omega_rad_s);
static void DriveStop(void);
static void ControlUpdate10ms(uint32_t now_ms);
static void PiCommandService(uint32_t now_ms);

static void ButtonService(uint32_t now_ms);
static void LedService(uint32_t now_ms);
static void SafetyCheck(uint32_t now_ms);

static void StateMachineRun(uint32_t now_ms);
static void StateMachineEnter(RobotState_t new_state, uint32_t now_ms);

static uint8_t TransportIsTargetValid(uint32_t now_ms);
static void TransportOnShovelIn(void);

/*==================== 电机底层 ====================*/

static void MotorWritePair(TIM_HandleTypeDef *htim, uint32_t ch_a, uint32_t ch_b,
                           int16_t command)
{
    uint32_t pulse;
    uint32_t arr = __HAL_TIM_GET_AUTORELOAD(htim);

    if (command > MOTOR_CMD_MAX) command = MOTOR_CMD_MAX;
    else if (command < MOTOR_CMD_MIN) command = MOTOR_CMD_MIN;

    if (command > 0)
    {
        pulse = ((uint32_t)command * arr) / MOTOR_CMD_MAX;
        __HAL_TIM_SET_COMPARE(htim, ch_a, pulse);
        __HAL_TIM_SET_COMPARE(htim, ch_b, 0);
    }
    else if (command < 0)
    {
        pulse = ((uint32_t)(-command) * arr) / MOTOR_CMD_MAX;
        __HAL_TIM_SET_COMPARE(htim, ch_a, 0);
        __HAL_TIM_SET_COMPARE(htim, ch_b, pulse);
    }
    else
    {
        __HAL_TIM_SET_COMPARE(htim, ch_a, 0);
        __HAL_TIM_SET_COMPARE(htim, ch_b, 0);
    }
}

static void MotorSetRaw(uint8_t motor_id, int16_t command)
{
    switch (motor_id)
    {
        case 1: MotorWritePair(&htim1, TIM_CHANNEL_1, TIM_CHANNEL_2, command); break;
        case 2: MotorWritePair(&htim1, TIM_CHANNEL_3, TIM_CHANNEL_4, command); break;
        case 3: MotorWritePair(&htim8, TIM_CHANNEL_1, TIM_CHANNEL_2, command); break;
        default: break;
    }
}

static void MotorInit(void)
{
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);

    for (uint8_t i = 0U; i < MOTOR_COUNT; i++)
    {
        MotorPid_Init(&g_motor_pid[i], PID_KP, PID_KI, PID_KD, PID_INTEGRAL_MAX);
    }

    MotorSetRaw(1, 0);
    MotorSetRaw(2, 0);
    MotorSetRaw(3, 0);
}

/*==================== 编码器 ====================*/

static void EncoderInit(void)
{
    HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);

    g_encoder_last[0] = __HAL_TIM_GET_COUNTER(&htim2);
    g_encoder_last[1] = (uint16_t)__HAL_TIM_GET_COUNTER(&htim3);
    g_encoder_last[2] = (uint16_t)__HAL_TIM_GET_COUNTER(&htim4);
}

static void EncoderUpdate(void)
{
    uint32_t now_1 = __HAL_TIM_GET_COUNTER(&htim2);
    uint16_t now_2 = (uint16_t)__HAL_TIM_GET_COUNTER(&htim3);
    uint16_t now_3 = (uint16_t)__HAL_TIM_GET_COUNTER(&htim4);

    g_encoder_delta[0] = (int32_t)(now_1 - g_encoder_last[0]);
    g_encoder_delta[1] = (int16_t)(now_2 - (uint16_t)g_encoder_last[1]);
    g_encoder_delta[2] = (int16_t)(now_3 - (uint16_t)g_encoder_last[2]);

    g_encoder_last[0] = now_1;
    g_encoder_last[1] = now_2;
    g_encoder_last[2] = now_3;
}

/*==================== 舵机 ====================*/

static void ServoInit(void)
{
    HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim10, TIM_CHANNEL_1);
    ServoSetAngle(SERVO_LEFT_INDEX, SERVO_ANGLE_INIT);
    ServoSetAngle(SERVO_RIGHT_INDEX, SERVO_ANGLE_INIT);
    ServoSetAngle(SERVO_CAM_INDEX, SERVO_ANGLE_INIT);
}

static void ServoSetAngle(uint8_t servo_id, uint16_t angle_x100)
{
    uint32_t pulse;
    uint32_t arr = __HAL_TIM_GET_AUTORELOAD(&htim9);

    if (angle_x100 > 18000U) angle_x100 = 18000U;

    /* 0°=500us, 180°=2500us, 50Hz 周期 20000us */
    pulse = 500U + ((uint32_t)angle_x100 * 2000U) / 18000U;
    pulse = (pulse * (arr + 1U)) / 20000U;

    switch (servo_id)
    {
        case SERVO_LEFT_INDEX:
            __HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_1, pulse);
            break;
        case SERVO_RIGHT_INDEX:
            __HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_2, pulse);
            break;
        case SERVO_CAM_INDEX:
            __HAL_TIM_SET_COMPARE(&htim10, TIM_CHANNEL_1, pulse);
            break;
        default:
            break;
    }
}

/*==================== 底盘驱动与闭环 ====================*/

static void DriveChassis(float vx_mm_s, float vy_mm_s, float omega_rad_s)
{
    ChassisSpeed_t chassis;
    int16_t motor_target[3];

    chassis.vx = vx_mm_s;
    chassis.vy = vy_mm_s;
    chassis.omega = omega_rad_s;

    Kinematics_ChassisToMotor(&chassis, motor_target);

    for (uint8_t i = 0U; i < MOTOR_COUNT; i++)
    {
        if (motor_target[i] > 2500) motor_target[i] = 2500;
        else if (motor_target[i] < -2500) motor_target[i] = -2500;
        g_target_counts[i] = motor_target[i];
    }
    g_ctrl_mode = CTRL_INTERNAL;
}

static void DriveStop(void)
{
    g_ctrl_mode = CTRL_IDLE;
    g_target_counts[0] = 0;
    g_target_counts[1] = 0;
    g_target_counts[2] = 0;
}

static void PiCommandService(uint32_t now_ms)
{
    PiMotorCmd_t cmd;

    if (PiProtocol_TakeMotorCmd(&g_pi, &cmd) == 0U)
    {
        return;
    }

    /* 待机/急停/铲铲时序/本地避障期间不接受树莓派接管 */
    if ((g_robot_state == ROBOT_STATE_IDLE) ||
        (g_robot_state == ROBOT_STATE_EMERGENCY_STOP) ||
        (g_robot_state == ROBOT_STATE_GRAB) ||
        (g_robot_state == ROBOT_STATE_RELEASE) ||
        (g_robot_state == ROBOT_STATE_AVOID) ||
        (g_robot_state == ROBOT_STATE_ERROR))
    {
        return;
    }

    g_pi_cmd_tick = now_ms;
    g_pi_cmd_ttl = cmd.ttl_ms;

    g_target_counts[0] = (int16_t)(((int32_t)cmd.motor_1 * PI_CMD_1000_COUNTS) / 1000);
    g_target_counts[1] = (int16_t)(((int32_t)cmd.motor_2 * PI_CMD_1000_COUNTS) / 1000);
    g_target_counts[2] = (int16_t)(((int32_t)cmd.motor_3 * PI_CMD_1000_COUNTS) / 1000);

    g_ctrl_mode = CTRL_PI;
}

static void ControlUpdate10ms(uint32_t now_ms)
{
    /* 树莓派命令 TTL 超时: 放弃外部控制 */
    if ((g_ctrl_mode == CTRL_PI) &&
        ((now_ms - g_pi_cmd_tick) > g_pi_cmd_ttl))
    {
        g_ctrl_mode = CTRL_IDLE;
    }

    /* 这些状态下底盘必须静止 */
    if ((g_robot_state == ROBOT_STATE_IDLE) ||
        (g_robot_state == ROBOT_STATE_EMERGENCY_STOP) ||
        (g_robot_state == ROBOT_STATE_GRAB) ||
        (g_robot_state == ROBOT_STATE_RELEASE) ||
        (g_robot_state == ROBOT_STATE_ERROR))
    {
        g_ctrl_mode = CTRL_IDLE;
    }

    for (uint8_t i = 0U; i < MOTOR_COUNT; i++)
    {
        int32_t delta = g_encoder_delta[i];

        if (delta > ENCODER_DELTA_LIMIT) delta = ENCODER_DELTA_LIMIT;
        else if (delta < -ENCODER_DELTA_LIMIT) delta = -ENCODER_DELTA_LIMIT;

        int16_t target = (g_ctrl_mode == CTRL_IDLE) ? 0 : g_target_counts[i];

        MotorPid_SetTarget(&g_motor_pid[i], target);
        MotorPid_Update(&g_motor_pid[i], (int16_t)delta, now_ms);

        int16_t output = MotorPid_GetOutput(&g_motor_pid[i]);
        MotorSetRaw((uint8_t)(i + 1U), output);

        debug_motor_target[i] = target;
        debug_motor_output[i] = output;
        debug_enc_delta[i] = delta;
    }
}

/*==================== 按键 ====================*/

static void ButtonService(uint32_t now_ms)
{
    GPIO_PinState raw;
    uint8_t raw_level;

    g_btn_exti_event = 0U;

    raw = HAL_GPIO_ReadPin(START_BUTTON_GPIO_Port, START_BUTTON_Pin);
    raw_level = (raw == GPIO_PIN_RESET) ? 0U : 1U;

    if (raw_level != g_btn_raw_level)
    {
        g_btn_raw_level = raw_level;
        g_btn_debounce_tick = now_ms;
    }
    else if ((uint32_t)(now_ms - g_btn_debounce_tick) >= BUTTON_DEBOUNCE_MS)
    {
        if (raw_level != g_btn_stable_level)
        {
            g_btn_stable_level = raw_level;

            if (raw_level == 0U)
            {
                g_btn_press_tick = now_ms;
                g_btn_long_fired = 0U;
            }
            else if (g_btn_long_fired == 0U)
            {
                g_btn_short_press = 1U;
            }
        }
    }

    if ((g_btn_stable_level == 0U) && (g_btn_long_fired == 0U) &&
        ((uint32_t)(now_ms - g_btn_press_tick) >= BUTTON_LONGPRESS_MS))
    {
        g_btn_long_press = 1U;
        g_btn_long_fired = 1U;
    }
}

/*==================== LED / 蜂鸣器 ====================*/

static void LedService(uint32_t now_ms)
{
    if (g_robot_state == ROBOT_STATE_EMERGENCY_STOP)
    {
        HAL_GPIO_WritePin(LED_STATUS_GPIO_Port, LED_STATUS_Pin, GPIO_PIN_SET);
        return;
    }

    if (g_robot_state == ROBOT_STATE_IDLE)
    {
        if ((uint32_t)(now_ms - g_led_tick) >= 500U)
        {
            g_led_tick = now_ms;
            HAL_GPIO_TogglePin(LED_STATUS_GPIO_Port, LED_STATUS_Pin);
        }
    }
    else
    {
        HAL_GPIO_WritePin(LED_STATUS_GPIO_Port, LED_STATUS_Pin, GPIO_PIN_SET);
    }
}

/*==================== 安全 ====================*/

static void SafetyCheck(uint32_t now_ms)
{
    if ((g_collision_left != 0U) || (g_collision_right != 0U))
    {
        if (g_robot_state != ROBOT_STATE_EMERGENCY_STOP)
        {
            StateMachineEnter(ROBOT_STATE_EMERGENCY_STOP, now_ms);
        }
        return;
    }

    if ((g_robot_state != ROBOT_STATE_IDLE) &&
        (g_robot_state != ROBOT_STATE_EMERGENCY_STOP) &&
        (g_pi.heartbeat_valid != 0U) &&
        ((uint32_t)(now_ms - g_pi.heartbeat_tick) > HEARTBEAT_TIMEOUT_MS))
    {
        StateMachineEnter(ROBOT_STATE_IDLE, now_ms);
    }
}

/*==================== 转运规则 ====================*/

static uint8_t TransportIsTargetValid(uint32_t now_ms)
{
    if (g_target.class_id == TARGET_CLASS_HAZARD)
    {
        return 0U;
    }

    if ((uint32_t)(now_ms - g_target.timestamp_ms) > DETECTION_TIMEOUT_MS)
    {
        return 0U;
    }

    /* 首次必须先单独转运普通物资 */
    if ((g_first_transport == 0U) && (g_target.class_id != TARGET_CLASS_SUPPLY))
    {
        return 0U;
    }

    /* 伤员必须单独转运 */
    if ((g_target.class_id == TARGET_CLASS_CASUALTY) &&
        ((g_stats.supply_count > 0U) || (g_stats.core_count > 0U)))
    {
        return 0U;
    }

    /* 一次最多 3 个物资 */
    if ((uint8_t)(g_stats.supply_count + g_stats.core_count) >= 3U)
    {
        return 0U;
    }

    return 1U;
}

static void TransportOnShovelIn(void)
{
    ServoSetAngle(SERVO_LEFT_INDEX, SHOVEL_ANGLE_DOWN);
    ServoSetAngle(SERVO_RIGHT_INDEX, SHOVEL_ANGLE_DOWN);

    switch (g_target.class_id)
    {
        case TARGET_CLASS_SUPPLY:
            g_stats.supply_count++;
            g_first_transport = 1U;
            break;
        case TARGET_CLASS_CORE:
            g_stats.core_count++;
            break;
        case TARGET_CLASS_CASUALTY:
            g_stats.casualty_count++;
            break;
        default:
            break;
    }
    g_target.loaded = 1U;
}

/*==================== 状态机 ====================*/

static void StateMachineEnter(RobotState_t new_state, uint32_t now_ms)
{
    g_robot_state = new_state;
    g_state_enter_tick = now_ms;
    g_state_start_x = g_odom.x_mm;
    g_state_start_y = g_odom.y_mm;
    debug_state = (uint32_t)new_state;

    switch (new_state)
    {
        case ROBOT_STATE_IDLE:
            DriveStop();
            break;

        case ROBOT_STATE_EMERGENCY_STOP:
            DriveStop();
            /* 声光报警 */
            HAL_GPIO_WritePin(LED_STATUS_GPIO_Port, LED_STATUS_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_SET);
            break;

        case ROBOT_STATE_GRAB:
            DriveStop();
            TransportOnShovelIn();
            break;

        case ROBOT_STATE_TRANSPORT:
            ServoSetAngle(SERVO_LEFT_INDEX, SHOVEL_ANGLE_HOLD);
            ServoSetAngle(SERVO_RIGHT_INDEX, SHOVEL_ANGLE_HOLD);
            break;

        case ROBOT_STATE_RELEASE:
            DriveStop();
            ServoSetAngle(SERVO_LEFT_INDEX, SHOVEL_ANGLE_UP);
            ServoSetAngle(SERVO_RIGHT_INDEX, SHOVEL_ANGLE_UP);
            g_target.loaded = 0U;
            break;

        default:
            break;
    }
}

static void StateMachineRun(uint32_t now_ms)
{
    uint32_t elapsed = now_ms - g_state_enter_tick;
    float traveled_mm;

    /* 长按: 解除急停封锁 */
    if (g_btn_long_press != 0U)
    {
        g_btn_long_press = 0U;
        if (g_robot_state == ROBOT_STATE_EMERGENCY_STOP)
        {
            g_collision_left = 0U;
            g_collision_right = 0U;
            HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_RESET);
            StateMachineEnter(ROBOT_STATE_IDLE, now_ms);
        }
    }

    /* 短按: 发车 / 取消 */
    if (g_btn_short_press != 0U)
    {
        g_btn_short_press = 0U;
        if (g_robot_state == ROBOT_STATE_IDLE)
        {
#if (APP_REQUIRE_PI_HEARTBEAT == 1)
            /* 视觉心跳未建立时不允许发车 */
            if ((g_pi.heartbeat_valid == 0U) ||
                ((uint32_t)(now_ms - g_pi.heartbeat_tick) > HEARTBEAT_TIMEOUT_MS))
            {
                /* 蜂鸣器短促提示: 阻塞 100ms, 仅在 IDLE 发生可接受 */
                HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_SET);
                HAL_Delay(100);
                HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_RESET);
                return;
            }
#endif
            Odometry_Reset(&g_odom);
            StateMachineEnter(ROBOT_STATE_SEARCH, now_ms);
        }
        else if (g_robot_state != ROBOT_STATE_EMERGENCY_STOP)
        {
            StateMachineEnter(ROBOT_STATE_IDLE, now_ms);
        }
    }

    switch (g_robot_state)
    {
        case ROBOT_STATE_IDLE:
            break;

        case ROBOT_STATE_SEARCH:
            if (g_pi.detection_valid != 0U)
            {
                PiDetection_t *det = &g_pi.latest_detection;
                g_target.class_id = det->class_id;
                g_target.distance_mm = det->distance_mm;
                g_target.center_x = det->center_x;
                g_target.timestamp_ms = det->timestamp_ms;

                if ((det->confidence >= DETECTION_CONFIDENCE_MIN) &&
                    (det->distance_mm > 0U) &&
                    (det->distance_mm <= DETECTION_DISTANCE_MAX) &&
                    (TransportIsTargetValid(now_ms) != 0U))
                {
                    StateMachineEnter(ROBOT_STATE_APPROACH, now_ms);
                    g_pi.detection_valid = 0U;
                    break;
                }
                g_pi.detection_valid = 0U;
            }

            if (g_ctrl_mode != CTRL_PI)
            {
                DriveChassis(0.0f, 0.0f, LOCAL_SEARCH_OMEGA);
            }

            if (elapsed >= SEARCH_TIMEOUT_MS)
            {
                StateMachineEnter(ROBOT_STATE_IDLE, now_ms);
            }
            break;

        case ROBOT_STATE_APPROACH:
            if ((g_distance_valid == 0x03U) &&
                (g_distance_left <= APPROACH_STOP_DIST_MM) &&
                (g_distance_right <= APPROACH_STOP_DIST_MM))
            {
                StateMachineEnter(ROBOT_STATE_GRAB, now_ms);
                break;
            }

            traveled_mm = Odometry_GetDistanceMm(&g_odom,
                                                 g_state_start_x, g_state_start_y);
            if (traveled_mm >= APPROACH_MAX_DIST_MM)
            {
                StateMachineEnter(ROBOT_STATE_SEARCH, now_ms);
                break;
            }

            if (elapsed >= APPROACH_TIMEOUT_MS)
            {
                StateMachineEnter(ROBOT_STATE_SEARCH, now_ms);
                break;
            }

            if (g_ctrl_mode != CTRL_PI)
            {
                DriveChassis(0.0f, LOCAL_APPROACH_VY, 0.0f);
            }
            break;

        case ROBOT_STATE_GRAB:
            if (elapsed >= TRANSPORT_PUSH_TIME_MS)
            {
                StateMachineEnter(ROBOT_STATE_TRANSPORT, now_ms);
            }
            break;

        case ROBOT_STATE_TRANSPORT:
            traveled_mm = Odometry_GetDistanceMm(&g_odom,
                                                 g_state_start_x, g_state_start_y);
            if (traveled_mm >= TRANSPORT_DIST_MM)
            {
                StateMachineEnter(ROBOT_STATE_RELEASE, now_ms);
                break;
            }
            if (elapsed >= TRANSPORT_TIMEOUT_MS)
            {
                StateMachineEnter(ROBOT_STATE_RELEASE, now_ms);
                break;
            }
            if (g_ctrl_mode != CTRL_PI)
            {
                DriveChassis(0.0f, LOCAL_TRANSPORT_VY, 0.0f);
            }
            break;

        case ROBOT_STATE_RELEASE:
            if (elapsed >= TRANSPORT_RELEASE_TIME_MS)
            {
                StateMachineEnter(ROBOT_STATE_RETURN, now_ms);
            }
            break;

        case ROBOT_STATE_RETURN:
            traveled_mm = Odometry_GetDistanceMm(&g_odom,
                                                 g_state_start_x, g_state_start_y);
            if (traveled_mm >= RETURN_DIST_MM)
            {
                StateMachineEnter(ROBOT_STATE_SEARCH, now_ms);
                break;
            }
            if (elapsed >= RETURN_TIMEOUT_MS)
            {
                StateMachineEnter(ROBOT_STATE_SEARCH, now_ms);
                break;
            }
            if (g_ctrl_mode != CTRL_PI)
            {
                DriveChassis(0.0f, LOCAL_RETURN_VY, 0.0f);
            }
            break;

        case ROBOT_STATE_AVOID:
            DriveChassis(0.0f, LOCAL_AVOID_VY, 0.0f);
            if (elapsed >= 500U)
            {
                StateMachineEnter(ROBOT_STATE_SEARCH, now_ms);
            }
            break;

        case ROBOT_STATE_EMERGENCY_STOP:
            break;

        case ROBOT_STATE_ERROR:
        default:
            DriveStop();
            break;
    }
}

/*==================== 对外接口 ====================*/

void App_Init(void)
{
    uint32_t boot_ms;

    MotorInit();
    EncoderInit();
    ServoInit();
    Kinematics_Init();
    Odometry_Init(&g_odom);

    /* 通信 (USART1 单字节中断接收) */
    PiProtocol_Init(&g_pi, &huart1);
    (void)PiProtocol_Start(&g_pi);

    /* 双激光测距 (USART2/3 DMA 循环接收, DMA 在 CubeMX Msp 中已 link) */
    Stp23l_Init(&g_stp_left, &huart2);
    Stp23l_Init(&g_stp_right, &huart3);
    (void)Stp23l_Start(&g_stp_left);
    (void)Stp23l_Start(&g_stp_right);

    /* 按键初始电平同步 */
    g_btn_raw_level = (HAL_GPIO_ReadPin(START_BUTTON_GPIO_Port, START_BUTTON_Pin) == GPIO_PIN_RESET)
                      ? 0U : 1U;
    g_btn_stable_level = g_btn_raw_level;

    boot_ms = HAL_GetTick();
    g_control_tick = boot_ms;
    g_safety_tick = boot_ms;
    g_telemetry_tick = boot_ms;
    g_led_tick = boot_ms;
    g_btn_debounce_tick = boot_ms;
    g_state_enter_tick = boot_ms;

    /* 最后启动看门狗: 4s IWDG (需在 hal_conf 中使能 HAL_IWDG_MODULE_ENABLED) */
    WDG_Init();
}

void App_Tick(void)
{
    uint32_t now_ms = HAL_GetTick();
    uint16_t dist;
    uint8_t valid_mask;

    /* 10ms: 编码器 → 里程计 → 速度环 PID → PWM */
    if ((uint32_t)(now_ms - g_control_tick) >= CONTROL_PERIOD_MS)
    {
        g_control_tick = now_ms;
        EncoderUpdate();
        Odometry_Update(&g_odom, g_encoder_delta, CONTROL_PERIOD_MS / 1000.0f);
        ControlUpdate10ms(now_ms);
    }

    /* 按键 / 状态机 / LED (电平轮询, 每轮执行) */
    ButtonService(now_ms);
    StateMachineRun(now_ms);
    LedService(now_ms);

    /* 50ms: 安全监督 */
    if ((uint32_t)(now_ms - g_safety_tick) >= SAFETY_CHECK_PERIOD_MS)
    {
        g_safety_tick = now_ms;
        SafetyCheck(now_ms);
    }

    /* 树莓派命令 */
    PiCommandService(now_ms);

    /* 激光测距 */
    Stp23l_Service(&g_stp_left);
    Stp23l_Service(&g_stp_right);

    valid_mask = 0U;
    if (Stp23l_GetDistance(&g_stp_left, now_ms, STP23L_MAX_AGE_MS, &dist) != 0U)
    {
        g_distance_left = dist;
        valid_mask |= 0x01U;
    }
    if (Stp23l_GetDistance(&g_stp_right, now_ms, STP23L_MAX_AGE_MS, &dist) != 0U)
    {
        g_distance_right = dist;
        valid_mask |= 0x02U;
    }
    g_distance_valid = valid_mask;
    debug_distance_left = g_distance_left;
    debug_distance_right = g_distance_right;

    /* 100ms: 遥测 */
    if ((uint32_t)(now_ms - g_telemetry_tick) >= TELEMETRY_PERIOD_MS)
    {
        g_telemetry_tick = now_ms;
        (void)PiProtocol_SendEncoderTelem(&g_pi,
                                          g_encoder_delta[0],
                                          g_encoder_delta[1],
                                          g_encoder_delta[2]);
        (void)PiProtocol_SendDistanceTelem(&g_pi,
                                           g_distance_left,
                                           g_distance_right,
                                           g_distance_valid);
    }

    debug_odom_x = g_odom.x_mm;
    debug_odom_y = g_odom.y_mm;
    debug_odom_theta = g_odom.theta_rad;
    debug_ctrl_mode = (uint32_t)g_ctrl_mode;

    WDG_Refresh();
    HAL_Delay(2);
}

/*==================== HAL 回调 (覆盖 weak 符号, 勿放 main.c) ====================*/

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == COLLISION_LEFT_Pin)
    {
        if (HAL_GPIO_ReadPin(COLLISION_LEFT_GPIO_Port, COLLISION_LEFT_Pin) == GPIO_PIN_RESET)
        {
            g_collision_left = 1U;
        }
    }
    else if (GPIO_Pin == COLLISION_RIGHT_Pin)
    {
        if (HAL_GPIO_ReadPin(COLLISION_RIGHT_GPIO_Port, COLLISION_RIGHT_Pin) == GPIO_PIN_RESET)
        {
            g_collision_right = 1U;
        }
    }
    else if (GPIO_Pin == START_BUTTON_Pin)
    {
        g_btn_exti_event = 1U;
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    PiProtocol_OnRxComplete(&g_pi, huart);
    Stp23l_OnRxComplete(&g_stp_left, huart);
    Stp23l_OnRxComplete(&g_stp_right, huart);
}

void HAL_UART_RxHalfCpltCallback(UART_HandleTypeDef *huart)
{
    Stp23l_OnRxHalfComplete(&g_stp_left, huart);
    Stp23l_OnRxHalfComplete(&g_stp_right, huart);
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    PiProtocol_OnUartError(&g_pi, huart);
    Stp23l_OnUartError(&g_stp_left, huart);
    Stp23l_OnUartError(&g_stp_right, huart);
}
