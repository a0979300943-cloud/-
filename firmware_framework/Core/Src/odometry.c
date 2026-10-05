/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    odometry.c
  * @brief   编码器里程计实现 (三轮全向轮)
  ******************************************************************************
  */
/* USER CODE END Header */

#include "odometry.h"
#include "kinematics.h"
#include <math.h>

/**
  * @brief  里程计初始化
  */
void Odometry_Init(Odometry_t *odom)
{
    odom->x_mm = 0.0f;
    odom->y_mm = 0.0f;
    odom->theta_rad = 0.0f;
    odom->vx_mm_s = 0.0f;
    odom->vy_mm_s = 0.0f;
    odom->omega_rad_s = 0.0f;
    odom->valid = 0U;
}

/**
  * @brief  里程计清零 (出发区发车时调用)
  */
void Odometry_Reset(Odometry_t *odom)
{
    odom->x_mm = 0.0f;
    odom->y_mm = 0.0f;
    odom->theta_rad = 0.0f;
    odom->vx_mm_s = 0.0f;
    odom->vy_mm_s = 0.0f;
    odom->omega_rad_s = 0.0f;
}

/**
  * @brief  角度归一化到 [-π, π]
  */
float Odometry_NormalizeAngle(float angle_rad)
{
    while (angle_rad > 3.14159265f)
        angle_rad -= 6.28318530f;
    while (angle_rad < -3.14159265f)
        angle_rad += 6.28318530f;
    return angle_rad;
}

/**
  * @brief  用三路编码器增量更新位姿
  *
  * 计算流程:
  *   1) 编码器计数增量 -> 各轮线位移 (毫米)
  *   2) 线位移/周期 -> 轮速 -> 正运动学 -> 底盘速度
  *   3) 底盘位移旋转到全局坐标后积分 (一阶欧拉)
  */
void Odometry_Update(Odometry_t *odom, const int32_t delta_counts[3], float dt_s)
{
    WheelSpeed_t wheel_vel;
    ChassisSpeed_t chassis_vel;

    if (dt_s < 1e-4f)
    {
        return;
    }

    /* 1) 计数 -> 轮子线速度 (mm/s) */
    wheel_vel.v1 = ((float)delta_counts[0] / KIN_ENCODER_COUNTS_PER_WHEEL_REV)
                   * KIN_WHEEL_CIRCUMFERENCE_MM / dt_s;
    wheel_vel.v2 = ((float)delta_counts[1] / KIN_ENCODER_COUNTS_PER_WHEEL_REV)
                   * KIN_WHEEL_CIRCUMFERENCE_MM / dt_s;
    wheel_vel.v3 = ((float)delta_counts[2] / KIN_ENCODER_COUNTS_PER_WHEEL_REV)
                   * KIN_WHEEL_CIRCUMFERENCE_MM / dt_s;

    /* 2) 正运动学: 轮速 -> 底盘速度 */
    Kinematics_WheelToChassis(&wheel_vel, &chassis_vel);

    /* 3) 本周期底盘系位移 */
    float dx_body = chassis_vel.vx * dt_s;
    float dy_body = chassis_vel.vy * dt_s;
    float dtheta = chassis_vel.omega * dt_s;

    /* 旋转到全局坐标并积分 */
    float cos_t = cosf(odom->theta_rad);
    float sin_t = sinf(odom->theta_rad);
    odom->x_mm += dx_body * cos_t - dy_body * sin_t;
    odom->y_mm += dx_body * sin_t + dy_body * cos_t;
    odom->theta_rad = Odometry_NormalizeAngle(odom->theta_rad + dtheta);

    /* 保存瞬时速度 */
    odom->vx_mm_s = chassis_vel.vx;
    odom->vy_mm_s = chassis_vel.vy;
    odom->omega_rad_s = chassis_vel.omega;
    odom->valid = 1U;
}

/**
  * @brief  当前位置相对参考点的平面距离 (毫米)
  */
float Odometry_GetDistanceMm(const Odometry_t *odom, float x0_mm, float y0_mm)
{
    float dx = odom->x_mm - x0_mm;
    float dy = odom->y_mm - y0_mm;
    return sqrtf(dx * dx + dy * dy);
}
