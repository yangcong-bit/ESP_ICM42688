/**
 * @file icm42688_dual.h
 * @brief 双 ICM-42688 融合解算库
 *
 * 功能:
 *   1. 双 IMU 同步读取 & 时间对齐
 *   2. 交叉校准 (补偿安装误差角)
 *   3. 加权互补滤波融合
 *   4. 陀螺仪偏置交叉补偿
 *   5. 异常值检测 & 降级运行
 */

#ifndef ICM42688_DUAL_H
#define ICM42688_DUAL_H

#include "icm42688.h"
#include "icm42688_alg.h"
#include "hardcore_eskf.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================
 *  3×3 旋转矩阵
 * ============================================================ */
typedef struct {
    float m[3][3];
} mat3_t;

/* ============================================================
 *  双 IMU 配置
 * ============================================================ */
typedef struct {
    /* 融合参数 */
    float   alpha;           /* 互补滤波器权重: 0~1, 越大越信任 IMU-A */
    float   kp;              /* Mahony 比例增益 */
    float   ki;              /* Mahony 积分增益 */
    float   sample_hz;       /* 采样频率 */
    bool    enable_misalign; /* 是否进行安装误差补偿 */
    bool    enable_cross_bias; /* 是否交叉补偿陀螺仪偏置 */

    /* IMU-B 相对 IMU-A 的安装偏差角 (度), 用于预补偿 */
    float   misalign_roll;   /* 绕 X 轴的偏差角 */
    float   misalign_pitch;  /* 绕 Y 轴的偏差角 */
    float   misalign_yaw;    /* 绕 Z 轴的偏差角 */
} dual_imu_cfg_t;

/* ============================================================
 *  单个 IMU 状态
 * ============================================================ */
typedef struct {
    icm42688_dev_t    *dev;           /* SPI 设备句柄 */
    icm42688_reading_t reading;       /* 最新读数 */
    icm42688_axis3f_t  gyro_bias_run; /* 运行时陀螺仪偏置 (在线估计, rad/s) */
    bool               valid;         /* 数据是否有效 */
    bool               online;        /* 传感器是否在线 */
    uint32_t           error_count;   /* 连续错误计数 */
} dual_imu_unit_t;

/* ============================================================
 *  融合结果
 * ============================================================ */
typedef struct {
    quat_t          quat;           /* 融合四元数 (极速模式, 不计算欧拉角) */
    icm42688_axis3f_t accel;        /* 融合加速度 (g) */
    icm42688_axis3f_t gyro;         /* 融合陀螺仪 (dps) */

    /* 置信度 & 诊断 */
    float           confidence;     /* 融合置信度 0~1 */
    float           accel_diff;     /* 两路加速度差值大小 (g) */
    float           gyro_diff;      /* 两路陀螺仪差值大小 (dps) */
    bool            cross_check_ok; /* 交叉校验是否通过 */
    uint64_t        timestamp_us;
} dual_imu_result_t;

/* ============================================================
 *  双 IMU 句柄
 * ============================================================ */
typedef struct {
    dual_imu_unit_t  imu[2];        /* IMU-A [0], IMU-B [1] */
    dual_imu_cfg_t   cfg;
    eskf_t           eskf_fused;           /* 融合后 ESKF 滤波器 */
    eskf_nominal_state_t eskf_state_fused; /* ESKF 标称状态 */
    uint64_t         last_update_us;       /* 上次 ESKF 更新时间 (用于计算 dt) */
    mat3_t           R_align;       /* 安装对齐旋转矩阵 */
    bool             initialized;
    uint32_t         fused_count;
} dual_imu_dev_t;

/* ============================================================
 *  API
 * ============================================================ */

/**
 * @brief 初始化双 IMU 系统
 * @param dev       双 IMU 句柄 (调用者分配)
 * @param imu_a     IMU-A 设备句柄 (已初始化)
 * @param imu_b     IMU-B 设备句柄 (已初始化)
 * @param cfg       融合配置, NULL = 默认值
 */
icm42688_err_t dual_imu_init(dual_imu_dev_t *dev,
                              icm42688_dev_t *imu_a,
                              icm42688_dev_t *imu_b,
                              const dual_imu_cfg_t *cfg);

/**
 * @brief 同步读取两路 IMU 并进行融合
 *
 * 内部执行:
 *   1. 同时读取 IMU-A 和 IMU-B
 *   2. 异常值检测 (差值超阈值则降级)
 *   3. 安装误差补偿 (旋转对齐)
 *   4. 交叉陀螺仪偏置补偿
 *   5. 加权融合
 *   6. Mahony AHRS 解算
 *
 * @param[out] result  融合输出
 */
icm42688_err_t dual_imu_update(dual_imu_dev_t *dev,
                                dual_imu_result_t *result);

/**
 * @brief 双 IMU 静态校准 (交叉校准)
 *
 * 在传感器静止时调用:
 *   - 分别采集两路偏移
 *   - 计算安装偏差旋转矩阵
 *   - 补偿陀螺仪交叉偏置
 *
 * @param samples 每路采样次数 (建议 ≥300)
 */
icm42688_err_t dual_imu_calibrate(dual_imu_dev_t *dev, uint32_t samples);

/**
 * @brief 手动设置安装偏差角
 */
void dual_imu_set_misalignment(dual_imu_dev_t *dev,
                                float roll_deg, float pitch_deg, float yaw_deg);

/**
 * @brief 获取当前融合置信度
 *   1.0 = 双 IMU 正常且一致
 *   0.5 = 仅单 IMU 工作
 *   0.0 = 双 IMU 均异常
 */
float dual_imu_get_confidence(const dual_imu_dev_t *dev);

/**
 * @brief 获取单路 IMU 状态
 */
const dual_imu_unit_t *dual_imu_get_unit(const dual_imu_dev_t *dev, int idx);

/**
 * @brief 重置融合滤波器
 */
void dual_imu_reset(dual_imu_dev_t *dev);

/* ============================================================
 *  辅助 — 3x3 矩阵运算
 * ============================================================ */
mat3_t mat3_identity(void);
mat3_t mat3_from_euler(float roll, float pitch, float yaw);
mat3_t mat3_mul(mat3_t a, mat3_t b);
icm42688_axis3f_t mat3_mul_vec(mat3_t m, icm42688_axis3f_t v);
mat3_t mat3_transpose(mat3_t m);

#ifdef __cplusplus
}
#endif

#endif /* ICM42688_DUAL_H */
