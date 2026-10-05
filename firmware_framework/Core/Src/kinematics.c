/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    kinematics.c
  * @brief   三轮全向轮运动学实现
  ******************************************************************************
  */
/* USER CODE END Header */

#include "kinematics.h"
#include <math.h>

/* 逆运动学矩阵 [3x3] */
static float kin_inv_matrix[3][3];

/* 正运动学矩阵 [3x3] */
static float kin_fwd_matrix[3][3];

/**
  * @brief  运动学初始化 - 计算矩阵
  */
void Kinematics_Init(void)
{
    float w1 = OMNI_WHEEL1_ANGLE;
    float w2 = OMNI_WHEEL2_ANGLE;
    float w3 = OMNI_WHEEL3_ANGLE;
    float R = OMNI_CHASSIS_RADIUS_MM;

    /* 逆运动学矩阵: v_wheel = [cos(w), sin(w), R] * [vx, vy, omega] */
    kin_inv_matrix[0][0] = cosf(w1);
    kin_inv_matrix[0][1] = sinf(w1);
    kin_inv_matrix[0][2] = R;

    kin_inv_matrix[1][0] = cosf(w2);
    kin_inv_matrix[1][1] = sinf(w2);
    kin_inv_matrix[1][2] = R;

    kin_inv_matrix[2][0] = cosf(w3);
    kin_inv_matrix[2][1] = sinf(w3);
    kin_inv_matrix[2][2] = R;

    /* 正运动学矩阵: 逆矩阵的伪逆 */
    /* 对于三轮全向轮，正运动学是逆矩阵的转置乘以 (J^T * J)^-1 */
    /* 简化计算: 三轮对称布局，逆矩阵可直接求得 */
    float inv[3][3] = {
        {cosf(w1), sinf(w1), R},
        {cosf(w2), sinf(w2), R},
        {cosf(w3), sinf(w3), R}
    };

    /* 计算 3x3 矩阵的逆 */
    float det = inv[0][0] * (inv[1][1] * inv[2][2] - inv[1][2] * inv[2][1])
              - inv[0][1] * (inv[1][0] * inv[2][2] - inv[1][2] * inv[2][0])
              + inv[0][2] * (inv[1][0] * inv[2][1] - inv[1][1] * inv[2][0]);

    if (fabsf(det) < 1e-6f)
    {
        /* 奇异矩阵，使用默认值 */
        det = 1.0f;
    }

    float inv_det = 1.0f / det;

    kin_fwd_matrix[0][0] = (inv[1][1] * inv[2][2] - inv[1][2] * inv[2][1]) * inv_det;
    kin_fwd_matrix[0][1] = (inv[0][2] * inv[2][1] - inv[0][1] * inv[2][2]) * inv_det;
    kin_fwd_matrix[0][2] = (inv[0][1] * inv[1][2] - inv[0][2] * inv[1][1]) * inv_det;

    kin_fwd_matrix[1][0] = (inv[1][2] * inv[2][0] - inv[1][0] * inv[2][2]) * inv_det;
    kin_fwd_matrix[1][1] = (inv[0][0] * inv[2][2] - inv[0][2] * inv[2][0]) * inv_det;
    kin_fwd_matrix[1][2] = (inv[0][2] * inv[1][0] - inv[0][0] * inv[1][2]) * inv_det;

    kin_fwd_matrix[2][0] = (inv[1][0] * inv[2][1] - inv[1][1] * inv[2][0]) * inv_det;
    kin_fwd_matrix[2][1] = (inv[0][1] * inv[2][0] - inv[0][0] * inv[2][1]) * inv_det;
    kin_fwd_matrix[2][2] = (inv[0][0] * inv[1][1] - inv[0][1] * inv[1][0]) * inv_det;
}

/**
  * @brief  逆运动学: 底盘速度 -> 轮速
  */
void Kinematics_ChassisToWheel(const ChassisSpeed_t *chassis, WheelSpeed_t *wheel)
{
    wheel->v1 = kin_inv_matrix[0][0] * chassis->vx +
                kin_inv_matrix[0][1] * chassis->vy +
                kin_inv_matrix[0][2] * chassis->omega;

    wheel->v2 = kin_inv_matrix[1][0] * chassis->vx +
                kin_inv_matrix[1][1] * chassis->vy +
                kin_inv_matrix[1][2] * chassis->omega;

    wheel->v3 = kin_inv_matrix[2][0] * chassis->vx +
                kin_inv_matrix[2][1] * chassis->vy +
                kin_inv_matrix[2][2] * chassis->omega;
}

/**
  * @brief  正运动学: 轮速 -> 底盘速度
  */
void Kinematics_WheelToChassis(const WheelSpeed_t *wheel, ChassisSpeed_t *chassis)
{
    chassis->vx = kin_fwd_matrix[0][0] * wheel->v1 +
                  kin_fwd_matrix[0][1] * wheel->v2 +
                  kin_fwd_matrix[0][2] * wheel->v3;

    chassis->vy = kin_fwd_matrix[1][0] * wheel->v1 +
                  kin_fwd_matrix[1][1] * wheel->v2 +
                  kin_fwd_matrix[1][2] * wheel->v3;

    chassis->omega = kin_fwd_matrix[2][0] * wheel->v1 +
                     kin_fwd_matrix[2][1] * wheel->v2 +
                     kin_fwd_matrix[2][2] * wheel->v3;
}

/**
  * @brief  轮速 -> 电机目标速度 (编码器增量/10ms)
  * @note   TI12 编码器模式 4 倍频: 轮子转 1 圈 = 500×4×30 = 60000 计数
  *         encoder_per_10ms = v_mm_s × COUNTS_PER_REV / 轮周长 × 0.01
  */
void Kinematics_WheelToMotor(const WheelSpeed_t *wheel, int16_t *motor_speed)
{
    motor_speed[0] = (int16_t)(wheel->v1 * KIN_VEL_TO_COUNTS_SCALE);
    motor_speed[1] = (int16_t)(wheel->v2 * KIN_VEL_TO_COUNTS_SCALE);
    motor_speed[2] = (int16_t)(wheel->v3 * KIN_VEL_TO_COUNTS_SCALE);
}

/**
  * @brief  电机实际速度 (编码器增量/10ms) -> 轮速 mm/s
  */
void Kinematics_MotorToWheel(const int16_t *motor_speed, WheelSpeed_t *wheel)
{
    wheel->v1 = (float)motor_speed[0] * KIN_COUNTS_TO_VEL_SCALE;
    wheel->v2 = (float)motor_speed[1] * KIN_COUNTS_TO_VEL_SCALE;
    wheel->v3 = (float)motor_speed[2] * KIN_COUNTS_TO_VEL_SCALE;
}

/**
  * @brief  底盘速度 -> 电机目标速度
  */
void Kinematics_ChassisToMotor(const ChassisSpeed_t *chassis, int16_t *motor_target)
{
    WheelSpeed_t wheel;
    Kinematics_ChassisToWheel(chassis, &wheel);
    Kinematics_WheelToMotor(&wheel, motor_target);
}

/**
  * @brief  直接控制接口: 给定底盘目标速度和角速度
  * @param  vx_mm_s: X方向速度 (mm/s), 前为正
  * @param  vy_mm_s: Y方向速度 (mm/s), 左为正
  * @param  omega_rad_s: 角速度 (rad/s), 逆时针为正
  * @param  motor1_out: 电机1目标 (编码器计数/10ms)
  * @param  motor2_out: 电机2目标 (编码器计数/10ms)
  * @param  motor3_out: 电机3目标 (编码器计数/10ms)
  * @note   满速约 2000 counts/10ms, 限幅 ±2500
  */
void Kinematics_SetTarget(float vx_mm_s, float vy_mm_s, float omega_rad_s,
                          int16_t *motor1_out, int16_t *motor2_out, int16_t *motor3_out)
{
    ChassisSpeed_t chassis;
    WheelSpeed_t wheel;
    int16_t motor_target[3];

    chassis.vx = vx_mm_s;
    chassis.vy = vy_mm_s;
    chassis.omega = omega_rad_s;

    Kinematics_ChassisToWheel(&chassis, &wheel);
    Kinematics_WheelToMotor(&wheel, motor_target);

    /* 限幅 (编码器计数/10ms) */
    for (int i = 0; i < 3; i++)
    {
        if (motor_target[i] > 2500)
            motor_target[i] = 2500;
        else if (motor_target[i] < -2500)
            motor_target[i] = -2500;
    }

    *motor1_out = motor_target[0];
    *motor2_out = motor_target[1];
    *motor3_out = motor_target[2];
}
