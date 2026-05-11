#ifndef HARDCORE_ESKF_H
#define HARDCORE_ESKF_H

#include <stdint.h>
#include <math.h>
#include "esp_attr.h"

/* 强制 16 字节对齐，满足 ESP32-S3 PIE 128-bit SIMD 总线要求 */
#ifndef ALIGN_16
#define ALIGN_16 __attribute__((aligned(16)))
#endif

/* 强制 16 字节对齐，利用 Xtensa 核心的总线带宽 */
typedef struct {
    float q[4];      // 标称四元数 [w, x, y, z]
    float bg[3];     // 陀螺仪零偏 (Bias) [x, y, z]
} __attribute__((aligned(16))) eskf_nominal_state_t;

typedef struct {
    float dx[6];     // 误差状态：[角度误差x, y, z, 零偏误差x, y, z]

    /* [F15 Task 2.1] P[6][8]: 每行 32 字节 (8×4), 保证所有行首地址 16 字节对齐
     * ESP32-S3 PIE 128-bit SIMD 要求操作数 16 字节对齐。
     * 算法中访问 P[i][j] 时, 循环仍使用 j < 6, 仅 padding 区域不使用。 */
    float P[6][8];   // 6x8 协方差矩阵 (有效区域 6x6, 每行 padding 2 float)
    
    // 系统噪声矩阵参数 (Q)
    float noise_gyro;
    float noise_bias;
    
    // 测量噪声矩阵参数 (R - 加速度计)
    float noise_accel;
} __attribute__((aligned(16))) eskf_t;

// 初始化滤波器
void eskf_init(eskf_t *eskf, eskf_nominal_state_t *nominal);

// 核心预测步
void eskf_predict(eskf_t *eskf, eskf_nominal_state_t *nominal, const float gyro[3], float dt);

// 核心更新步
void eskf_update_accel(eskf_t *eskf, eskf_nominal_state_t *nominal, const float accel[3]);

#endif
