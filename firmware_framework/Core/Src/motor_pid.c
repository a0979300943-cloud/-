/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    motor_pid.c
  * @brief   电机 PID 闭环控制实现
  ******************************************************************************
  */
/* USER CODE END Header */

#include "motor_pid.h"

/**
  * @brief  PID 控制器初始化
  */
void Pid_Init(PidController_t *pid, float kp, float ki, float kd,
               float integral_max, float output_max)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->integral_max = integral_max;
    pid->output_max = output_max;

    Pid_Reset(pid);
}

/**
  * @brief  PID 重置
  */
void Pid_Reset(PidController_t *pid)
{
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
    pid->prev_measurement = 0.0f;
    pid->first_run = 1U;
    pid->output = 0.0f;
}

/**
  * @brief  PID 更新
  * @param  setpoint: 设定值
  * @param  measurement: 测量值
  * @return PID 输出
  */
float Pid_Update(PidController_t *pid, float setpoint, float measurement)
{
    float error = setpoint - measurement;

    /* 比例项 */
    float p_term = pid->kp * error;

    /* 积分项 (带抗饱和) */
    pid->integral += error;
    if (pid->integral > pid->integral_max)
        pid->integral = pid->integral_max;
    else if (pid->integral < -pid->integral_max)
        pid->integral = -pid->integral_max;
    float i_term = pid->ki * pid->integral;

    /* 微分项 (对测量值微分，避免设定值突变导致微分冲击) */
    float d_term = 0.0f;
    if (pid->first_run == 0U)
    {
        float d_measurement = pid->prev_measurement - measurement;
        d_term = pid->kd * d_measurement;
    }
    pid->prev_measurement = measurement;
    pid->prev_error = error;
    pid->first_run = 0U;

    /* 总输出 */
    pid->output = p_term + i_term + d_term;

    /* 输出限幅 */
    if (pid->output > pid->output_max)
        pid->output = pid->output_max;
    else if (pid->output < -pid->output_max)
        pid->output = -pid->output_max;

    return pid->output;
}

/**
  * @brief  电机 PID 初始化
  */
void MotorPid_Init(MotorPid_t *mpid, float kp, float ki, float kd,
                    float integral_max)
{
    Pid_Init(&mpid->pid, kp, ki, kd, integral_max, 1000.0f);
    mpid->target_speed = 0;
    mpid->actual_speed = 0;
    mpid->output_cmd = 0;
    mpid->last_update_ms = 0U;
}

/**
  * @brief  电机 PID 更新 (10ms 周期调用)
  * @param  actual_speed: 实际速度 (编码器增量/10ms)
  */
void MotorPid_Update(MotorPid_t *mpid, int16_t actual_speed, uint32_t now_ms)
{
    mpid->actual_speed = actual_speed;
    mpid->last_update_ms = now_ms;

    float output = Pid_Update(&mpid->pid,
                               (float)mpid->target_speed,
                               (float)actual_speed);
    mpid->output_cmd = (int16_t)output;
}

/**
  * @brief  设置目标速度
  */
void MotorPid_SetTarget(MotorPid_t *mpid, int16_t target_speed)
{
    mpid->target_speed = target_speed;
}

/**
  * @brief  获取输出命令
  */
int16_t MotorPid_GetOutput(MotorPid_t *mpid)
{
    return mpid->output_cmd;
}
