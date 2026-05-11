#include "hardcore_eskf.h"
#include "dspm_mult.h"
#include "esp_dsp.h"
#include <string.h>

void eskf_init(eskf_t *eskf, eskf_nominal_state_t *nominal) {
    nominal->q[0] = 1.0f;
    nominal->q[1] = 0.0f;
    nominal->q[2] = 0.0f;
    nominal->q[3] = 0.0f;
    memset(nominal->bg, 0, sizeof(nominal->bg));
    memset(eskf->dx, 0, sizeof(eskf->dx));
    memset(eskf->P, 0, sizeof(eskf->P));
    for (int i = 0; i < 6; i++) eskf->P[i][i] = 0.01f;
    eskf->noise_gyro  = 0.001f;
    eskf->noise_bias  = 0.0001f;
    eskf->noise_accel = 0.05f;
}

/* 预测步: P = F * P * F^T + Q (SIMD 加速) */
/* [F15 Task 2.3] 移除 IRAM_ATTR: sqrtf() 调用 Flash 中的标准库,
 * 若 Cache 关闭 (Flash 擦写) 会触发 Cache Panic。
 * 依赖 -O3 指令 Cache 即可, 不需要常驻 IRAM。 */
void eskf_predict(eskf_t *eskf, eskf_nominal_state_t *nominal, const float gyro[3], float dt) {
    float wx = gyro[0] - nominal->bg[0];
    float wy = gyro[1] - nominal->bg[1];
    float wz = gyro[2] - nominal->bg[2];

    float q_w = nominal->q[0], q_x = nominal->q[1];
    float q_y = nominal->q[2], q_z = nominal->q[3];
    float half_dt = 0.5f * dt;

    nominal->q[0] += half_dt * (-q_x*wx - q_y*wy - q_z*wz);
    nominal->q[1] += half_dt * ( q_w*wx - q_z*wy + q_y*wz);
    nominal->q[2] += half_dt * ( q_z*wx + q_w*wy - q_x*wz);
    nominal->q[3] += half_dt * (-q_y*wx + q_x*wy + q_w*wz);

    /* 归一化 */
    ALIGN_16 float q_buf[4] = { nominal->q[0], nominal->q[1], nominal->q[2], nominal->q[3] };
    float qn = sqrtf(q_buf[0]*q_buf[0] + q_buf[1]*q_buf[1] + q_buf[2]*q_buf[2] + q_buf[3]*q_buf[3]);
    if (qn > 1e-8f) {
        float inv_n = 1.0f / qn;
        nominal->q[0] *= inv_n; nominal->q[1] *= inv_n;
        nominal->q[2] *= inv_n; nominal->q[3] *= inv_n;
    }

    /* 构建 F (6x6), 强制 16 字节对齐 */
    ALIGN_16 float F[6][6] = {0};
    for (int i = 0; i < 6; i++) F[i][i] = 1.0f;
    F[0][1] =  wz * dt;  F[0][2] = -wy * dt;  F[0][3] = -dt;
    F[1][0] = -wz * dt;  F[1][2] =  wx * dt;  F[1][4] = -dt;
    F[2][0] =  wy * dt;  F[2][1] = -wx * dt;  F[2][5] = -dt;

    /* [F15 Task 2.1] P[6][8] stride 修正: dspm_mult_f32 需要连续 6 列数据,
     * 但 P 每行有 8 列 (padding). 先拷贝到局部 6×6 缓冲再做矩阵乘。 */
    ALIGN_16 float P_flat[6][6];
    for (int i = 0; i < 6; i++)
        memcpy(P_flat[i], eskf->P[i], 6 * sizeof(float));

    /* P_next = F * P (SIMD 矩阵乘, 对齐) */
    ALIGN_16 float P_next[6][6];
    dspm_mult_f32((float *)F, (float *)P_flat, (float *)P_next, 6, 6, 6);

    /* F^T (对齐) */
    ALIGN_16 float F_T[6][6];
    for (int i = 0; i < 6; i++)
        for (int j = 0; j < 6; j++)
            F_T[i][j] = F[j][i];

    /* P = P_next * F^T (SIMD 矩阵乘) */
    dspm_mult_f32((float *)P_next, (float *)F_T, (float *)P_flat, 6, 6, 6);

    /* 写回 P[6][8] (仅前 6 列) */
    for (int i = 0; i < 6; i++)
        memcpy(eskf->P[i], P_flat[i], 6 * sizeof(float));

    /* 过程噪声 Q */
    float dt2 = dt * dt;
    eskf->P[0][0] += eskf->noise_gyro * dt2;
    eskf->P[1][1] += eskf->noise_gyro * dt2;
    eskf->P[2][2] += eskf->noise_gyro * dt2;
    eskf->P[3][3] += eskf->noise_bias * dt;
    eskf->P[4][4] += eskf->noise_bias * dt;
    eskf->P[5][5] += eskf->noise_bias * dt;

    /* 强制对称 */
    for (int i = 0; i < 6; i++)
        for (int j = i + 1; j < 6; j++)
            eskf->P[i][j] = eskf->P[j][i] =
                (eskf->P[i][j] + eskf->P[j][i]) * 0.5f;

    /* [审查优化] 对角线微小正向注入: 防止长时 1000Hz 浮点积分
     * 后 P 矩阵因浮点吸收 (Swamping) 失去正定性 */
    for (int i = 0; i < 6; i++) {
        eskf->P[i][i] += 1e-6f;
    }
}

/* 更新步: 序贯更新 (SIMD 加速) */
/* [F15 Task 2.3] 移除 IRAM_ATTR (同上, sqrtf 依赖 Flash Cache) */
void eskf_update_accel(eskf_t *eskf, eskf_nominal_state_t *nominal, const float accel[3]) {
    if (isnan(accel[0]) || isnan(accel[1]) || isnan(accel[2])) return;
    float a_norm = sqrtf(accel[0]*accel[0] + accel[1]*accel[1] + accel[2]*accel[2]);
    if (a_norm < 0.1f) return;
    if (a_norm > 16.0f) return;
    float inv_a = 1.0f / a_norm;
    float z[3] = { accel[0]*inv_a, accel[1]*inv_a, accel[2]*inv_a };

    float qw = nominal->q[0], qx = nominal->q[1];
    float qy = nominal->q[2], qz = nominal->q[3];
    float g_pred[3] = {
        2.0f * (qx*qz - qw*qy),
        2.0f * (qy*qz + qw*qx),
        qw*qw - qx*qx - qy*qy + qz*qz
    };

    float y[3] = { z[0] - g_pred[0], z[1] - g_pred[1], z[2] - g_pred[2] };

    ALIGN_16 float H[3][6] = {
        { 0.0f,      -g_pred[2],  g_pred[1], 0.0f, 0.0f, 0.0f },
        { g_pred[2],  0.0f,      -g_pred[0], 0.0f, 0.0f, 0.0f },
        {-g_pred[1],  g_pred[0],  0.0f,      0.0f, 0.0f, 0.0f }
    };

    for (int axis = 0; axis < 3; axis++) {
        ALIGN_16 float h_row[6];
        for (int j = 0; j < 6; j++) h_row[j] = H[axis][j];

        /* PHt = P * H^T: 逐行点积 (SIMD, 对齐) */
        ALIGN_16 float PHt[6];
        for (int i = 0; i < 6; i++) {
            dsps_dotprod_f32(&eskf->P[i][0], h_row, &PHt[i], 6);
        }

        /* S = h · PHt + R (SIMD 点积) */
        float s_tmp;
        dsps_dotprod_f32(h_row, PHt, &s_tmp, 6);
        float S = eskf->noise_accel + s_tmp;

        /* K = PHt / S (对齐) */
        float inv_S = 1.0f / S;
        ALIGN_16 float K[6];
        for (int i = 0; i < 6; i++) K[i] = PHt[i] * inv_S;

        /* innovation = y - H * dx (SIMD 点积) */
        float h_dx;
        dsps_dotprod_f32(h_row, eskf->dx, &h_dx, 6);
        float innovation = y[axis] - h_dx;

        /* dx += K * innovation */
        for (int i = 0; i < 6; i++) eskf->dx[i] += K[i] * innovation;

        /* Hp = H * P: 1x6 × 6x6 = 1x6 (行主序标量展开) */
        float Hp[6];
        for (int j = 0; j < 6; j++) {
            Hp[j] = h_row[0]*eskf->P[0][j] + h_row[1]*eskf->P[1][j]
                  + h_row[2]*eskf->P[2][j] + h_row[3]*eskf->P[3][j]
                  + h_row[4]*eskf->P[4][j] + h_row[5]*eskf->P[5][j];
        }

        /* P -= K ⊗ Hp (外积, 6x6) */
        for (int i = 0; i < 6; i++) {
            float ki = K[i];
            for (int j = 0; j < 6; j++) {
                eskf->P[i][j] -= ki * Hp[j];
            }
        }
    }

    /* 误差状态重置 */
    float dq[4] = { 1.0f, 0.5f*eskf->dx[0], 0.5f*eskf->dx[1], 0.5f*eskf->dx[2] };
    float q0 = nominal->q[0], q1 = nominal->q[1];
    float q2 = nominal->q[2], q3 = nominal->q[3];
    nominal->q[0] = q0*dq[0] - q1*dq[1] - q2*dq[2] - q3*dq[3];
    nominal->q[1] = q0*dq[1] + q1*dq[0] + q2*dq[3] - q3*dq[2];
    nominal->q[2] = q0*dq[2] - q1*dq[3] + q2*dq[0] + q3*dq[1];
    nominal->q[3] = q0*dq[3] + q1*dq[2] - q2*dq[1] + q3*dq[0];

    /* 归一化 */
    ALIGN_16 float q_buf[4] = { nominal->q[0], nominal->q[1], nominal->q[2], nominal->q[3] };
    float qn = sqrtf(q_buf[0]*q_buf[0] + q_buf[1]*q_buf[1] + q_buf[2]*q_buf[2] + q_buf[3]*q_buf[3]);
    if (qn > 1e-8f) {
        float inv_n = 1.0f / qn;
        nominal->q[0] *= inv_n; nominal->q[1] *= inv_n;
        nominal->q[2] *= inv_n; nominal->q[3] *= inv_n;
    }

    nominal->bg[0] += eskf->dx[3];
    nominal->bg[1] += eskf->dx[4];
    nominal->bg[2] += eskf->dx[5];
    memset(eskf->dx, 0, sizeof(eskf->dx));
}
