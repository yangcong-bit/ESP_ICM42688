/**
 * @file icm42688_alg.h
 * @brief ICM-42688-P 姿态解算算法库
 *
 * 提供:
 *   1. Mahony 互补滤波器 (AHRS) — 低资源消耗, 适合实时嵌入式
 *   2. 欧拉角 / 四元数 互转工具
 *   3. 简易滑动平均滤波
 */

#ifndef ICM42688_ALG_H
#define ICM42688_ALG_H

#include <stdint.h>
#include <stdbool.h>
#include "icm42688.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================
 *  四元数
 * ============================================================ */
typedef struct {
    float w, x, y, z;
} quat_t;

/* ============================================================
 *  欧拉角 (度)
 * ============================================================ */
typedef struct {
    float roll;   /* 绕 X 轴 (前后翻) */
    float pitch;  /* 绕 Y 轴 (左右俯仰) */
    float yaw;    /* 绕 Z 轴 (偏航) */
} euler_t;

/* ============================================================
 *  Mahony AHRS 滤波器
 * ============================================================ */
typedef struct {
    /* 滤波器状态 */
    quat_t  q;             /* 当前四元数 (归一化) */
    float   kp;            /* 比例增益 (加速度计修正) */
    float   ki;            /* 积分增益 (陀螺仪偏置补偿) */
    float   integral_fb[3]; /* 积分反馈 */
    float   gyro_bias[3];  /* 在线估计的陀螺仪偏置 (rad/s) */
    float   sample_period; /* 采样周期 (s) */
    bool    first_run;     /* 首次运行标志 */
} mahony_t;

/* ============================================================
 *  API — Mahony 滤波器
 * ============================================================ */

/**
 * @brief 初始化 Mahony 滤波器
 * @param filt       滤波器句柄
 * @param kp         比例增益 (典型值: 0.5 ~ 2.0, 默认 1.0)
 * @param ki         积分增益 (典型值: 0.0 ~ 0.1, 默认 0.005)
 * @param sample_hz  采样频率 (Hz), 与 IMU ODR 匹配
 */
void mahony_init(mahony_t *filt, float kp, float ki, float sample_hz);

/**
 * @brief 更新一次 (每次 IMU 数据就绪时调用)
 *
 * @param filt       滤波器句柄
 * @param accel      加速度计 (g, 重力轴为 +Z)
 * @param gyro       陀螺仪 (dps)
 */
void mahony_update(mahony_t *filt,
                   const icm42688_axis3f_t *accel,
                   const icm42688_axis3f_t *gyro);

/**
 * @brief 获取当前四元数
 */
quat_t mahony_get_quat(const mahony_t *filt);

/**
 * @brief 获取当前欧拉角 (度)
 */
euler_t mahony_get_euler(const mahony_t *filt);

/**
 * @brief 重置滤波器到初始状态
 */
void mahony_reset(mahony_t *filt);

/* ============================================================
 *  API — 四元数 / 欧拉角 互转
 * ============================================================ */

/**
 * @brief 四元数 → 欧拉角 (度)
 *        旋转顺序: ZYX (Yaw → Pitch → Roll)
 *        与 NED 坐标系兼容
 */
euler_t quat_to_euler(quat_t q);

/**
 * @brief 欧拉角 (度) → 四元数
 */
quat_t euler_to_quat(euler_t euler);

/**
 * @brief 归一化四元数
 */
quat_t quat_normalize(quat_t q);

/**
 * @brief 四元数乘法 (Hamilton product)
 */
quat_t quat_mul(quat_t a, quat_t b);

/**
 * @brief 四元数共轭
 */
quat_t quat_conjugate(quat_t q);

/**
 * @brief 旋转一个三维向量 (用四元数)
 */
icm42688_axis3f_t quat_rotate_vec(quat_t q, icm42688_axis3f_t v);

/* ============================================================
 *  API — 滑动平均滤波
 * ============================================================ */
#define MAHONY_MAX_FILTER_LEN  32

typedef struct {
    float buffer[MAHONY_MAX_FILTER_LEN];
    int   idx;
    int   len;
    float sum;
} moving_avg_t;

/**
 * @brief 初始化滑动平均
 * @param window_size 窗口长度 (1 ~ MAHONY_MAX_FILTER_LEN)
 */
void moving_avg_init(moving_avg_t *ma, int window_size);

/**
 * @brief 输入一个新值, 返回滤波后的值
 */
float moving_avg_push(moving_avg_t *ma, float new_val);

/**
 * @brief 重置
 */
void moving_avg_reset(moving_avg_t *ma);

/**
 * @brief 简易低通滤波 (一阶 IIR)
 *
 *   y[n] = alpha * x[n] + (1-alpha) * y[n-1]
 */
typedef struct {
    float y_prev;
    float alpha;  /* 0~1, 越小越平滑 */
    bool  first;
} lowpass_t;

void  lowpass_init(lowpass_t *lp, float alpha);
float lowpass_push(lowpass_t *lp, float x);
void  lowpass_reset(lowpass_t *lp);

/* ============================================================
 *  API — 重力方向解算 (简单互补滤波)
 * ============================================================ */

/**
 * @brief 使用加速度计估计 roll/pitch (仅当加速度计可信时)
 *
 *   roll  = atan2(ay, az)
 *   pitch = atan2(-ax, sqrt(ay² + az²))
 *
 * 注意: 此方法不含 yaw, 且在动态环境下不准确
 */
euler_t accel_to_euler(const icm42688_axis3f_t *accel);

#ifdef __cplusplus
}
#endif

#endif /* ICM42688_ALG_H */
