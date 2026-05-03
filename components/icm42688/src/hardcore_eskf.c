#include "hardcore_eskf.h"
#include <string.h>

void eskf_init(eskf_t *eskf, eskf_nominal_state_t *nominal) {
    // 初始姿态设为单位四元数
    nominal->q[0] = 1.0f;
    nominal->q[1] = 0.0f;
    nominal->q[2] = 0.0f;
    nominal->q[3] = 0.0f;
    
    memset(nominal->bg, 0, sizeof(nominal->bg));
    memset(eskf->dx, 0, sizeof(eskf->dx));
    memset(eskf->P, 0, sizeof(eskf->P));

    // 初始化协方差矩阵 P 为对角阵
    for (int i = 0; i < 6; i++) {
        eskf->P[i][i] = 0.01f;
    }

    // 调参区：根据你那两颗背靠背传感器的底噪进行设置
    eskf->noise_gyro = 0.001f;   // 陀螺仪白噪声方差
    eskf->noise_bias = 0.0001f;  // 陀螺仪零偏随机游走方差
    eskf->noise_accel = 0.05f;   // 加速度计噪声方差
}

// 预测步：用陀螺仪积分，更新名义状态和协方差
void IRAM_ATTR eskf_predict(eskf_t *eskf, eskf_nominal_state_t *nominal, const float gyro[3], float dt) {
    // 1. 扣除零偏后的真实角速度
    float wx = gyro[0] - nominal->bg[0];
    float wy = gyro[1] - nominal->bg[1];
    float wz = gyro[2] - nominal->bg[2];
    
    // 2. 名义状态更新 (四元数四阶龙格库塔或一阶积分，这里用极速一阶近似)
    float q_w = nominal->q[0], q_x = nominal->q[1], q_y = nominal->q[2], q_z = nominal->q[3];
    float half_dt = 0.5f * dt;
    
    nominal->q[0] += half_dt * (-q_x*wx - q_y*wy - q_z*wz);
    nominal->q[1] += half_dt * ( q_w*wx - q_z*wy + q_y*wz);
    nominal->q[2] += half_dt * ( q_z*wx + q_w*wy - q_x*wz);
    nominal->q[3] += half_dt * (-q_y*wx + q_x*wy + q_w*wz);
    
    // 四元数归一化
    float norm = 1.0f / sqrtf(nominal->q[0]*nominal->q[0] + nominal->q[1]*nominal->q[1] + 
                              nominal->q[2]*nominal->q[2] + nominal->q[3]*nominal->q[3]);
    nominal->q[0] *= norm; nominal->q[1] *= norm; nominal->q[2] *= norm; nominal->q[3] *= norm;

    // 3. 误差协方差矩阵 P 的更新: P = F * P * F^T + Q
    // 极度硬核：将 6x6 稀疏矩阵 F 直接展开运算，不调用任何矩阵库！
    // F = [ I - [w x]dt , -I dt ]
    //     [ 0           ,   I   ]
    float F[6][6] = {0};
    for(int i=0; i<6; i++) F[i][i] = 1.0f;
    F[0][1] =  wz * dt; F[0][2] = -wy * dt; F[0][3] = -dt;
    F[1][0] = -wz * dt; F[1][2] =  wx * dt; F[1][4] = -dt;
    F[2][0] =  wy * dt; F[2][1] = -wx * dt; F[2][5] = -dt;

    float P_next[6][6] = {0};
    // 考虑到 F 的强稀疏性，实际工程中这里可以进一步展开成纯标量相乘，这里保留一层轻量级循环以兼顾可读性
    for(int i=0; i<6; i++) {
        for(int j=0; j<6; j++) {
            float sum = 0.0f;
            for(int k=0; k<6; k++) {
                sum += F[i][k] * eskf->P[k][j];
            }
            P_next[i][j] = sum;
        }
    }
    
    for(int i=0; i<6; i++) {
        for(int j=0; j<6; j++) {
            float sum = 0.0f;
            for(int k=0; k<6; k++) {
                sum += P_next[i][k] * F[j][k]; // F^T
            }
            eskf->P[i][j] = sum;
        }
    }

    // 加入过程噪声 Q
    eskf->P[0][0] += eskf->noise_gyro * dt * dt;
    eskf->P[1][1] += eskf->noise_gyro * dt * dt;
    eskf->P[2][2] += eskf->noise_gyro * dt * dt;
    eskf->P[3][3] += eskf->noise_bias * dt;
    eskf->P[4][4] += eskf->noise_bias * dt;
    eskf->P[5][5] += eskf->noise_bias * dt;
    
    // 致命细节：防发散保护，强制矩阵对称
    for(int i=0; i<6; i++) {
        for(int j=i+1; j<6; j++) {
            eskf->P[i][j] = eskf->P[j][i] = (eskf->P[i][j] + eskf->P[j][i]) * 0.5f;
        }
    }
}

// 测量步：利用加速度计校准 Pitch 和 Roll (利用序贯更新，斩断矩阵求逆)
void IRAM_ATTR eskf_update_accel(eskf_t *eskf, eskf_nominal_state_t *nominal, const float accel[3]) {
    // 归一化加速度测量值
    float a_norm = sqrtf(accel[0]*accel[0] + accel[1]*accel[1] + accel[2]*accel[2]);
    if (a_norm < 0.1f) return; // 自由落体保护
    float inv_a = 1.0f / a_norm;
    float z[3] = {accel[0]*inv_a, accel[1]*inv_a, accel[2]*inv_a};

    // 基于当前四元数预测的重力方向 (在体坐标系下)
    float q_w = nominal->q[0], q_x = nominal->q[1], q_y = nominal->q[2], q_z = nominal->q[3];
    float g_pred[3] = {
        2.0f * (q_x*q_z - q_w*q_y),
        2.0f * (q_y*q_z + q_w*q_x),
        q_w*q_w - q_x*q_x - q_y*q_y + q_z*q_z
    };

    // 观测残差
    float y[3] = { z[0] - g_pred[0], z[1] - g_pred[1], z[2] - g_pred[2] };

    // 观测矩阵的雅可比 H (只针对角度误差部分有非零值)
    // H = [ [g_pred x] , 0_3x3 ]
    float H[3][6] = {
        { 0.0f,      -g_pred[2],  g_pred[1], 0.0f, 0.0f, 0.0f },
        { g_pred[2],  0.0f,      -g_pred[0], 0.0f, 0.0f, 0.0f },
        {-g_pred[1],  g_pred[0],  0.0f,      0.0f, 0.0f, 0.0f }
    };

    // ---- 序贯更新 (Sequential Update) ----
    // 把 3 维观测拆成 3 次标量更新，完美避开 3x3 矩阵求逆！
    for (int axis = 0; axis < 3; axis++) {
        // 计算标量 S = H_i * P * H_i^T + R
        float S = eskf->noise_accel;
        float PHt[6] = {0}; // P * H_i^T
        for (int i = 0; i < 6; i++) {
            for (int j = 0; j < 6; j++) {
                PHt[i] += eskf->P[i][j] * H[axis][j];
            }
            S += H[axis][i] * PHt[i];
        }

        // 标量除法求增益 K (6x1 向量)
        float inv_S = 1.0f / S;
        float K[6];
        for (int i = 0; i < 6; i++) {
            K[i] = PHt[i] * inv_S;
        }

        // 更新状态 dx = dx + K * (y - H*dx)
        float h_dx = 0.0f;
        for (int i = 0; i < 6; i++) h_dx += H[axis][i] * eskf->dx[i];
        float innovation = y[axis] - h_dx;
        for (int i = 0; i < 6; i++) eskf->dx[i] += K[i] * innovation;

        // 更新协方差 P = P - K * H_i * P
        float K_H[6][6];
        for (int i = 0; i < 6; i++) {
            for (int j = 0; j < 6; j++) {
                K_H[i][j] = K[i] * H[axis][j];
            }
        }
        
        float P_new[6][6] = {0};
        for (int i = 0; i < 6; i++) {
            for (int j = 0; j < 6; j++) {
                float sum = eskf->P[i][j];
                for (int k = 0; k < 6; k++) {
                    sum -= K_H[i][k] * eskf->P[k][j];
                }
                P_new[i][j] = sum;
            }
        }
        memcpy(eskf->P, P_new, sizeof(P_new));
    }

    // ---- 误差状态重置 (Reset) ----
    // 1. 用算出的 dx 更新名义四元数 (小角度近似)
    float dq[4] = { 1.0f, 0.5f*eskf->dx[0], 0.5f*eskf->dx[1], 0.5f*eskf->dx[2] };
    
    // 四元数乘法 q_new = nominal_q * dq
    float q0 = nominal->q[0], q1 = nominal->q[1], q2 = nominal->q[2], q3 = nominal->q[3];
    nominal->q[0] = q0*dq[0] - q1*dq[1] - q2*dq[2] - q3*dq[3];
    nominal->q[1] = q0*dq[1] + q1*dq[0] + q2*dq[3] - q3*dq[2];
    nominal->q[2] = q0*dq[2] - q1*dq[3] + q2*dq[0] + q3*dq[1];
    nominal->q[3] = q0*dq[3] + q1*dq[2] - q2*dq[1] + q3*dq[0];

    // 重新归一化
    float norm = 1.0f / sqrtf(nominal->q[0]*nominal->q[0] + nominal->q[1]*nominal->q[1] + 
                              nominal->q[2]*nominal->q[2] + nominal->q[3]*nominal->q[3]);
    nominal->q[0] *= norm; nominal->q[1] *= norm; nominal->q[2] *= norm; nominal->q[3] *= norm;

    // 2. 更新陀螺仪零偏
    nominal->bg[0] += eskf->dx[3];
    nominal->bg[1] += eskf->dx[4];
    nominal->bg[2] += eskf->dx[5];

    // 3. 清零误差状态，等待下一轮积分
    memset(eskf->dx, 0, sizeof(eskf->dx));
}
