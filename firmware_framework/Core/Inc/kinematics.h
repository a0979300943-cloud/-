/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    kinematics.h
  * @brief   三轮全向轮运动学头文件
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef KINEMATICS_H
#define KINEMATICS_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"

/* 全向轮布局参数 */
#define OMNI_WHEEL_RADIUS_MM    33.0f   /* 轮子半径 (毫米) */
#define OMNI_CHASSIS_RADIUS_MM  80.0f   /* 底盘中心到轮子中心距离 (毫米) */

/* 轮子安装角度 (弧度，从正X轴逆时针) */
#define OMNI_WHEEL1_ANGLE       0.0f          /* 0° 正前方 */
#define OMNI_WHEEL2_ANGLE       2.094395f     /* 120° */
#define OMNI_WHEEL3_ANGLE       4.188790f     /* 240° */

/* 编码器/减速组物理参数 (实测后修改) */
#define KIN_ENCODER_PPR             500.0f    /* 编码器每转每通道脉冲数 */
#define KIN_ENCODER_QUAD_X4         4.0f      /* STM32 TI12 编码器模式 4 倍频 */
#define KIN_MOTOR_GEAR_RATIO        30.0f     /* 电机减速比 */
#define KIN_WHEEL_DIAMETER_MM       66.0f     /* 轮子直径 (毫米) */

/* 轮子转一圈对应的编码器计数 = 500 × 4 × 30 = 60000 */
#define KIN_ENCODER_COUNTS_PER_WHEEL_REV \
    (KIN_ENCODER_PPR * KIN_ENCODER_QUAD_X4 * KIN_MOTOR_GEAR_RATIO)

/* 轮子周长 (毫米) */
#define KIN_WHEEL_CIRCUMFERENCE_MM \
    (3.14159265f * KIN_WHEEL_DIAMETER_MM)

/* mm/s 与 编码器计数/10ms 的换算尺度 */
#define KIN_VEL_TO_COUNTS_SCALE \
    (KIN_ENCODER_COUNTS_PER_WHEEL_REV * 0.01f / KIN_WHEEL_CIRCUMFERENCE_MM)
#define KIN_COUNTS_TO_VEL_SCALE \
    (KIN_WHEEL_CIRCUMFERENCE_MM / (KIN_ENCODER_COUNTS_PER_WHEEL_REV * 0.01f))

/* 速度结构体 (底盘坐标系) */
typedef struct
{
    float vx;                   /* X方向速度 (mm/s) */
    float vy;                   /* Y方向速度 (mm/s) */
    float omega;                /* 角速度 (rad/s) */
} ChassisSpeed_t;

/* 轮速结构体 */
typedef struct
{
    float v1;                   /* 轮子1线速度 (mm/s) */
    float v2;                   /* 轮子2线速度 (mm/s) */
    float v3;                   /* 轮子3线速度 (mm/s) */
} WheelSpeed_t;

/* 接口函数 */
void Kinematics_Init(void);

/* 逆运动学: 底盘速度 -> 轮速 */
void Kinematics_ChassisToWheel(const ChassisSpeed_t *chassis, WheelSpeed_t *wheel);

/* 正运动学: 轮速 -> 底盘速度 */
void Kinematics_WheelToChassis(const WheelSpeed_t *wheel, ChassisSpeed_t *chassis);

/* 轮速 -> 电机转速 (编码器值/10ms) */
void Kinematics_WheelToMotor(const WheelSpeed_t *wheel, int16_t *motor_speed);

/* 电机转速 -> 轮速 */
void Kinematics_MotorToWheel(const int16_t *motor_speed, WheelSpeed_t *wheel);

/* 底盘速度 -> 电机目标速度 */
void Kinematics_ChassisToMotor(const ChassisSpeed_t *chassis, int16_t *motor_target);

/* 直接控制接口: 给定底盘目标速度和角速度 */
void Kinematics_SetTarget(float vx_mm_s, float vy_mm_s, float omega_rad_s,
                          int16_t *motor1_out, int16_t *motor2_out, int16_t *motor3_out);

#ifdef __cplusplus
}
#endif

#endif /* KINEMATICS_H */
