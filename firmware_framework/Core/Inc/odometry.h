/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    odometry.h
  * @brief   编码器里程计头文件 (三轮全向轮)
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef ODOMETRY_H
#define ODOMETRY_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"

/* 里程计位姿与速度 */
typedef struct
{
    /* 全局坐标位姿 */
    float x_mm;                 /* X 坐标 (毫米，前向为正) */
    float y_mm;                 /* Y 坐标 (毫米，左向为正) */
    float theta_rad;            /* 航向角 (弧度，逆时针为正, -π~π) */

    /* 底盘坐标系瞬时速度 */
    float vx_mm_s;              /* 前向速度 mm/s */
    float vy_mm_s;              /* 侧向速度 mm/s */
    float omega_rad_s;          /* 角速度 rad/s */

    /* 状态 */
    uint8_t valid;              /* 首次更新后置 1 */
} Odometry_t;

/* 接口函数 */
void Odometry_Init(Odometry_t *odom);
void Odometry_Reset(Odometry_t *odom);

/**
  * @brief  用三路编码器增量更新里程计
  * @param  delta_counts: 10ms 周期内三路编码器增量 (编码器计数)
  * @param  dt_s: 采样周期 (秒), 通常为 0.01
  */
void Odometry_Update(Odometry_t *odom, const int32_t delta_counts[3], float dt_s);

/* 工具函数 */
float Odometry_GetDistanceMm(const Odometry_t *odom, float x0_mm, float y0_mm);
float Odometry_NormalizeAngle(float angle_rad);

#ifdef __cplusplus
}
#endif

#endif /* ODOMETRY_H */
