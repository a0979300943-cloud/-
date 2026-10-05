/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    motor_pid.h
  * @brief   电机 PID 闭环控制头文件
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef MOTOR_PID_H
#define MOTOR_PID_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"

/* PID 控制器结构体 */
typedef struct
{
    /* 参数 */
    float kp;                   /* 比例系数 */
    float ki;                   /* 积分系数 */
    float kd;                   /* 微分系数 */
    float integral_max;         /* 积分上限 */
    float output_max;           /* 输出上限 */

    /* 状态 */
    float integral;             /* 积分累计 */
    float prev_error;           /* 上次误差 */
    float prev_measurement;     /* 上次测量值 (用于微分) */
    uint8_t first_run;          /* 首次运行标志 */

    /* 输出 */
    float output;               /* 当前输出 */
} PidController_t;

/* 电机速度闭环结构体 */
typedef struct
{
    PidController_t pid;        /* PID 控制器 */
    int16_t  target_speed;      /* 目标速度 (编码器增量/10ms) */
    int16_t  actual_speed;      /* 实际速度 (编码器增量/10ms) */
    int16_t  output_cmd;        /* 输出命令 (-1000~1000) */
    uint32_t last_update_ms;    /* 上次更新时间 */
} MotorPid_t;

/* 接口函数 */
void Pid_Init(PidController_t *pid, float kp, float ki, float kd,
               float integral_max, float output_max);
float Pid_Update(PidController_t *pid, float setpoint, float measurement);
void Pid_Reset(PidController_t *pid);

void MotorPid_Init(MotorPid_t *mpid, float kp, float ki, float kd,
                    float integral_max);
void MotorPid_Update(MotorPid_t *mpid, int16_t actual_speed, uint32_t now_ms);
void MotorPid_SetTarget(MotorPid_t *mpid, int16_t target_speed);
int16_t MotorPid_GetOutput(MotorPid_t *mpid);

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_PID_H */
