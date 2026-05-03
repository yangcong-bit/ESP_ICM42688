#ifndef HARDCORE_ESKF_H
#define HARDCORE_ESKF_H

#include <stdint.h>
#include <math.h>
#include "esp_attr.h"

/* 强制 16 字节对齐，利用 Xtensa 核心的总线带宽 */
typedef struct {
    float q[4];      // 标称四元数 [w, x, y, z]
    float bg[3];     // 陀螺仪零偏 (Bias) [x, y, z]
} __attribute__((aligned(16))) eskf_nominal_state_t;

typedef struct {
    float dx[6];     // 误差状态：[角度误差x, y, z, 零偏误差x, y, z]
    float P[6][6];   // 6x6 协方差矩阵
    
    // 系统噪声矩阵参数 (Q)
    float noise_gyro;
    float noise_bias;
    
    // 测量噪声矩阵参数 (R - 加速度计)
    float noise_accel;
} __attribute__((aligned(16))) eskf_t;

// 初始化滤波器
void eskf_init(eskf_t *eskf, eskf_nominal_state_t *nominal);

// 核心预测步 (跑在 IRAM，传入融合后的陀螺仪数据)
void IRAM_ATTR eskf_predict(eskf_t *eskf, eskf_nominal_state_t *nominal, const float gyro[3], float dt);

// 核心更新步 (跑在 IRAM，序贯更新，彻底消灭矩阵求逆)
void IRAM_ATTR eskf_update_accel(eskf_t *eskf, eskf_nominal_state_t *nominal, const float accel[3]);

#endif
