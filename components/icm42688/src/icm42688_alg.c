/**
 * @file icm42688_alg.c
 * @brief ICM-42688-P 姿态解算算法实现
 *
 * 基于 Mahony AHRS 互补滤波器 + 四元数积分
 */

#include "icm42688_alg.h"
#include <math.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

#define DEG2RAD(d) ((d) * M_PI / 180.0f)
#define RAD2DEG(r) ((r) * 180.0f / M_PI)

/* ============================================================
 *  四元数工具
 * ============================================================ */

quat_t quat_normalize(quat_t q)
{
    float n = sqrtf(q.w*q.w + q.x*q.x + q.y*q.y + q.z*q.z);
    if (n < 1e-10f) {
        /* 返回单位四元数 */
        return (quat_t){1, 0, 0, 0};
    }
    float inv = 1.0f / n;
    q.w *= inv;
    q.x *= inv;
    q.y *= inv;
    q.z *= inv;
    return q;
}

quat_t quat_mul(quat_t a, quat_t b)
{
    return (quat_t){
        .w = a.w*b.w - a.x*b.x - a.y*b.y - a.z*b.z,
        .x = a.w*b.x + a.x*b.w + a.y*b.z - a.z*b.y,
        .y = a.w*b.y - a.x*b.z + a.y*b.w + a.z*b.x,
        .z = a.w*b.z + a.x*b.y - a.y*b.x + a.z*b.w,
    };
}

quat_t quat_conjugate(quat_t q)
{
    return (quat_t){ .w = q.w, .x = -q.x, .y = -q.y, .z = -q.z };
}

icm42688_axis3f_t quat_rotate_vec(quat_t q, icm42688_axis3f_t v)
{
    /* v' = q * v * q^{-1} */
    quat_t qv = { .w = 0, .x = v.x, .y = v.y, .z = v.z };
    quat_t q_conj = quat_conjugate(q);
    quat_t tmp = quat_mul(q, qv);
    quat_t result = quat_mul(tmp, q_conj);
    return (icm42688_axis3f_t){ result.x, result.y, result.z };
}

/* ============================================================
 *  四元数 ↔ 欧拉角
 * ============================================================ */

euler_t quat_to_euler(quat_t q)
{
    euler_t e;

    /* Roll (X) */
    float sinr_cosp = 2.0f * (q.w * q.x + q.y * q.z);
    float cosr_cosp = 1.0f - 2.0f * (q.x * q.x + q.y * q.y);
    e.roll = RAD2DEG(atan2f(sinr_cosp, cosr_cosp));

    /* Pitch (Y) — 限制在 ±90° */
    float sinp = 2.0f * (q.w * q.y - q.z * q.x);
    if (fabsf(sinp) >= 1.0f) {
        e.pitch = RAD2DEG(copysignf(M_PI / 2.0f, sinp)); /* gimbal lock */
    } else {
        e.pitch = RAD2DEG(asinf(sinp));
    }

    /* Yaw (Z) */
    float siny_cosp = 2.0f * (q.w * q.z + q.x * q.y);
    float cosy_cosp = 1.0f - 2.0f * (q.y * q.y + q.z * q.z);
    e.yaw = RAD2DEG(atan2f(siny_cosp, cosy_cosp));

    return e;
}

quat_t euler_to_quat(euler_t euler)
{
    float roll  = DEG2RAD(euler.roll);
    float pitch = DEG2RAD(euler.pitch);
    float yaw   = DEG2RAD(euler.yaw);

    float cr = cosf(roll * 0.5f);
    float sr = sinf(roll * 0.5f);
    float cp = cosf(pitch * 0.5f);
    float sp = sinf(pitch * 0.5f);
    float cy = cosf(yaw * 0.5f);
    float sy = sinf(yaw * 0.5f);

    return (quat_t){
        .w = cr * cp * cy + sr * sp * sy,
        .x = sr * cp * cy - cr * sp * sy,
        .y = cr * sp * cy + sr * cp * sy,
        .z = cr * cp * sy - sr * sp * cy,
    };
}

/* ============================================================
 *  Mahony AHRS
 * ============================================================ */

void mahony_init(mahony_t *filt, float kp, float ki, float sample_hz)
{
    memset(filt, 0, sizeof(*filt));
    filt->q = (quat_t){ .w = 1, .x = 0, .y = 0, .z = 0 };
    filt->kp = kp;
    filt->ki = ki;
    filt->sample_period = (sample_hz > 0) ? (1.0f / sample_hz) : 0.001f;
    filt->first_run = true;
}

void mahony_reset(mahony_t *filt)
{
    filt->q = (quat_t){ .w = 1, .x = 0, .y = 0, .z = 0 };
    filt->integral_fb[0] = 0;
    filt->integral_fb[1] = 0;
    filt->integral_fb[2] = 0;
    filt->gyro_bias[0] = 0;
    filt->gyro_bias[1] = 0;
    filt->gyro_bias[2] = 0;
    filt->first_run = true;
}

void mahony_update(mahony_t *filt,
                   const icm42688_axis3f_t *accel,
                   const icm42688_axis3f_t *gyro)
{
    float ax = accel->x, ay = accel->y, az = accel->z;
    float gx = DEG2RAD(gyro->x);  /* rad/s */
    float gy = DEG2RAD(gyro->y);
    float gz = DEG2RAD(gyro->z);
    float dt = filt->sample_period;

    float qw = filt->q.w, qx = filt->q.x, qy = filt->q.y, qz = filt->q.z;

    /* 归一化加速度计 */
    float norm_a = sqrtf(ax*ax + ay*ay + az*az);
    if (norm_a < 0.01f) {
        /* 加速度计数据无效, 仅用陀螺仪积分 */
        goto gyro_only;
    }
    ax /= norm_a;
    ay /= norm_a;
    az /= norm_a;

    /* 估算重力在机体坐标系下的方向: R(q)^T * [0,0,1] */
    float vx = 2.0f * (qx*qz - qw*qy);
    float vy = 2.0f * (qw*qx + qy*qz);
    float vz = 1.0f - 2.0f * (qx*qx + qy*qy);

    /* 误差 = 加速度计 × 估算重力方向 (叉积) */
    float ex = ay*vz - az*vy;
    float ey = az*vx - ax*vz;
    float ez = ax*vy - ay*vx;

    /* 积分反馈 (用于估计陀螺仪偏置) */
    if (filt->ki > 0.0f) {
        filt->integral_fb[0] += filt->ki * ex * dt;
        filt->integral_fb[1] += filt->ki * ey * dt;
        filt->integral_fb[2] += filt->ki * ez * dt;
        gx += filt->integral_fb[0];
        gy += filt->integral_fb[1];
        gz += filt->integral_fb[2];
    }

    /* 比例反馈修正 */
    gx += filt->kp * ex;
    gy += filt->kp * ey;
    gz += filt->kp * ez;

gyro_only:
    ;
    /* 四元数微分方程: dq/dt = 0.5 * q ⊗ ω */
    float qwDot = 0.5f * (-qx*gx - qy*gy - qz*gz);
    float qxDot = 0.5f * ( qw*gx + qy*gz - qz*gy);
    float qyDot = 0.5f * ( qw*gy - qx*gz + qz*gx);
    float qzDot = 0.5f * ( qw*gz + qx*gy - qy*gx);

    /* 一阶积分 */
    qw += qwDot * dt;
    qx += qxDot * dt;
    qy += qyDot * dt;
    qz += qzDot * dt;

    /* 归一化 */
    float inv_n = 1.0f / sqrtf(qw*qw + qx*qx + qy*qy + qz*qz);
    filt->q.w = qw * inv_n;
    filt->q.x = qx * inv_n;
    filt->q.y = qy * inv_n;
    filt->q.z = qz * inv_n;

    /* 保存在线陀螺仪偏置估计 */
    filt->gyro_bias[0] = filt->integral_fb[0];
    filt->gyro_bias[1] = filt->integral_fb[1];
    filt->gyro_bias[2] = filt->integral_fb[2];
}

quat_t mahony_get_quat(const mahony_t *filt)
{
    return filt->q;
}

euler_t mahony_get_euler(const mahony_t *filt)
{
    return quat_to_euler(filt->q);
}

/* ============================================================
 *  加速度计 → Roll / Pitch
 * ============================================================ */
euler_t accel_to_euler(const icm42688_axis3f_t *accel)
{
    euler_t e = {0};
    e.roll  = RAD2DEG(atan2f(accel->y, accel->z));
    e.pitch = RAD2DEG(atan2f(-accel->x,
                       sqrtf(accel->y*accel->y + accel->z*accel->z)));
    e.yaw = 0;  /* 无法确定 yaw */
    return e;
}

/* ============================================================
 *  滑动平均滤波
 * ============================================================ */
void moving_avg_init(moving_avg_t *ma, int window_size)
{
    memset(ma, 0, sizeof(*ma));
    if (window_size < 1) window_size = 1;
    if (window_size > MAHONY_MAX_FILTER_LEN) window_size = MAHONY_MAX_FILTER_LEN;
    ma->len = window_size;
}

float moving_avg_push(moving_avg_t *ma, float new_val)
{
    /* 减去最旧值 */
    ma->sum -= ma->buffer[ma->idx];
    /* 加上最新值 */
    ma->buffer[ma->idx] = new_val;
    ma->sum += new_val;
    /* 移动索引 */
    ma->idx = (ma->idx + 1) % ma->len;
    return ma->sum / (float)ma->len;
}

void moving_avg_reset(moving_avg_t *ma)
{
    memset(ma, 0, sizeof(*ma));
}

/* ============================================================
 *  一阶低通 IIR
 * ============================================================ */
void lowpass_init(lowpass_t *lp, float alpha)
{
    lp->y_prev = 0;
    lp->alpha = alpha;
    lp->first = true;
}

float lowpass_push(lowpass_t *lp, float x)
{
    if (lp->first) {
        lp->y_prev = x;
        lp->first = false;
    }
    lp->y_prev = lp->alpha * x + (1.0f - lp->alpha) * lp->y_prev;
    return lp->y_prev;
}

void lowpass_reset(lowpass_t *lp)
{
    lp->y_prev = 0;
    lp->first = true;
}
