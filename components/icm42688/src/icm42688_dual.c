/**
 * @file icm42688_dual.c
 * @brief 双 ICM-42688 融合解算实现
 */

#include "icm42688_dual.h"
#include "esp_log.h"
#include "esp_timer.h"
#include <math.h>
#include <string.h>

static const char *TAG = "dual_imu";

/* ============================================================
 *  异常检测阈值
 * ============================================================ */
#define ACCEL_DIFF_THRESHOLD_G     0.5f   /* 加速度差值超过此值报警 (g) */
#define GYRO_DIFF_THRESHOLD_DPS   10.0f   /* 陀螺仪差值超过此值报警 (dps) */
#define MAX_ERROR_COUNT            10      /* 连续错误次数上限, 超过标记离线 */

/* ============================================================
 *  3×3 矩阵运算
 * ============================================================ */

mat3_t mat3_identity(void)
{
    mat3_t r;
    memset(&r, 0, sizeof(r));
    r.m[0][0] = r.m[1][1] = r.m[2][2] = 1.0f;
    return r;
}

mat3_t mat3_mul(mat3_t a, mat3_t b)
{
    mat3_t r;
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            r.m[i][j] = 0;
            for (int k = 0; k < 3; k++) {
                r.m[i][j] += a.m[i][k] * b.m[k][j];
            }
        }
    }
    return r;
}

icm42688_axis3f_t mat3_mul_vec(mat3_t m, icm42688_axis3f_t v)
{
    icm42688_axis3f_t r;
    r.x = m.m[0][0]*v.x + m.m[0][1]*v.y + m.m[0][2]*v.z;
    r.y = m.m[1][0]*v.x + m.m[1][1]*v.y + m.m[1][2]*v.z;
    r.z = m.m[2][0]*v.x + m.m[2][1]*v.y + m.m[2][2]*v.z;
    return r;
}

mat3_t mat3_transpose(mat3_t m)
{
    mat3_t r;
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            r.m[i][j] = m.m[j][i];
        }
    }
    return r;
}

/**
 * @brief 欧拉角 (度) → 旋转矩阵 R_ZYX
 *
 *   R = Rz(yaw) * Ry(pitch) * Rx(roll)
 */
mat3_t mat3_from_euler(float roll_deg, float pitch_deg, float yaw_deg)
{
    float r = roll_deg  * M_PI / 180.0f;
    float p = pitch_deg * M_PI / 180.0f;
    float y = yaw_deg   * M_PI / 180.0f;

    float cr = cosf(r), sr = sinf(r);
    float cp = cosf(p), sp = sinf(p);
    float cy = cosf(y), sy = sinf(y);

    mat3_t R;
    R.m[0][0] = cy*cp;
    R.m[0][1] = cy*sp*sr - sy*cr;
    R.m[0][2] = cy*sp*cr + sy*sr;
    R.m[1][0] = sy*cp;
    R.m[1][1] = sy*sp*sr + cy*cr;
    R.m[1][2] = sy*sp*cr - cy*sr;
    R.m[2][0] = -sp;
    R.m[2][1] = cp*sr;
    R.m[2][2] = cp*cr;
    return R;
}

/* ============================================================
 *  辅助 — 向量运算
 * ============================================================ */
static float vec3_norm(icm42688_axis3f_t v)
{
    return sqrtf(v.x*v.x + v.y*v.y + v.z*v.z);
}

static float vec3_diff_norm(icm42688_axis3f_t a, icm42688_axis3f_t b)
{
    float dx = a.x - b.x;
    float dy = a.y - b.y;
    float dz = a.z - b.z;
    return sqrtf(dx*dx + dy*dy + dz*dz);
}

/* ============================================================
 *  API — 初始化
 * ============================================================ */
icm42688_err_t dual_imu_init(dual_imu_dev_t *dev,
                              icm42688_dev_t *imu_a,
                              icm42688_dev_t *imu_b,
                              const dual_imu_cfg_t *cfg)
{
    if (!dev || !imu_a || !imu_b) return ICM42688_ERR_CONFIG;

    memset(dev, 0, sizeof(*dev));

    /* 默认配置 */
    dual_imu_cfg_t def_cfg = {
        .alpha            = 0.5f,     /* 等权融合 */
        .kp               = 1.0f,
        .ki               = 0.005f,
        .sample_hz        = 1000.0f,
        .enable_misalign  = true,
        .enable_cross_bias= true,
        .misalign_roll    = 0.0f,
        .misalign_pitch   = 0.0f,
        .misalign_yaw     = 0.0f,
    };
    if (cfg) {
        def_cfg = *cfg;
    }
    dev->cfg = def_cfg;

    /* 关联设备 */
    dev->imu[0].dev   = imu_a;
    dev->imu[0].online = true;
    dev->imu[1].dev   = imu_b;
    dev->imu[1].online = true;

    /* 计算安装对齐旋转矩阵 */
    if (dev->cfg.enable_misalign) {
        dev->R_align = mat3_from_euler(
            dev->cfg.misalign_roll,
            dev->cfg.misalign_pitch,
            dev->cfg.misalign_yaw);
    } else {
        dev->R_align = mat3_identity();
    }

    /* 初始化 6-状态误差卡尔曼滤波器 (ESKF) */
    eskf_init(&dev->eskf_fused, &dev->eskf_state_fused);
    dev->last_update_us = esp_timer_get_time();

    dev->initialized = true;

    ESP_LOGI(TAG, "Dual IMU initialized: alpha=%.2f, misalign=[%.1f, %.1f, %.1f]°",
             dev->cfg.alpha,
             dev->cfg.misalign_roll, dev->cfg.misalign_pitch, dev->cfg.misalign_yaw);

    return ICM42688_OK;
}

/* ============================================================
 *  API — 双路同步读取 + 融合
 * ============================================================ */
icm42688_err_t dual_imu_update(dual_imu_dev_t *dev,
                                dual_imu_result_t *result)
{
    if (!dev || !dev->initialized || !result) return ICM42688_ERR_NOT_INIT;

    /* ---- 1. 双路 SPI 真正并发: queue_trans(A+B) → wait(A) → wait(B) ---- */
    icm42688_err_t err_a = icm42688_queue_read(dev->imu[0].dev);
    icm42688_err_t err_b = icm42688_queue_read(dev->imu[1].dev);
    /* 此时 SPI2 和 SPI3 在硬件层面同时传输, CPU 不阻塞 */
    /* 收割 A 的 DMA 结果 */
    if (err_a == ICM42688_OK) {
        err_a = icm42688_wait_read(dev->imu[0].dev, &dev->imu[0].reading);
    }
    /* 收割 B 的 DMA 结果 */
    if (err_b == ICM42688_OK) {
        err_b = icm42688_wait_read(dev->imu[1].dev, &dev->imu[1].reading);
    }

    /* 检测在线状态 */
    if (err_a == ICM42688_OK) {
        dev->imu[0].online = true;
        dev->imu[0].error_count = 0;
    } else {
        dev->imu[0].error_count++;
        if (dev->imu[0].error_count >= MAX_ERROR_COUNT) {
            dev->imu[0].online = false;
            ESP_LOGW(TAG, "IMU-A offline after %lu errors", (unsigned long)dev->imu[0].error_count);
        }
    }

    if (err_b == ICM42688_OK) {
        dev->imu[1].online = true;
        dev->imu[1].error_count = 0;
    } else {
        dev->imu[1].error_count++;
        if (dev->imu[1].error_count >= MAX_ERROR_COUNT) {
            dev->imu[1].online = false;
            ESP_LOGW(TAG, "IMU-B offline after %lu errors", (unsigned long)dev->imu[1].error_count);
        }
    }

    /* 至少需要一路在线 */
    if (!dev->imu[0].online && !dev->imu[1].online) {
        result->confidence = 0.0f;
        result->cross_check_ok = false;
        return ICM42688_ERR_TIMEOUT;
    }

    /* ---- 2. 选择有效数据 ---- */
    icm42688_axis3f_t accel_a = dev->imu[0].reading.accel_g;
    icm42688_axis3f_t gyro_a  = dev->imu[0].reading.gyro_dps;
    icm42688_axis3f_t accel_b = dev->imu[1].reading.accel_g;
    icm42688_axis3f_t gyro_b  = dev->imu[1].reading.gyro_dps;

    /* IMU-B 安装误差补偿 (将 B 的数据旋转到 A 的坐标系) */
    if (dev->cfg.enable_misalign) {
        accel_b = mat3_mul_vec(dev->R_align, accel_b);
        gyro_b  = mat3_mul_vec(dev->R_align, gyro_b);
    }

    /* ---- 3. 交叉校验 & 异常检测 ---- */
    result->accel_diff = vec3_diff_norm(accel_a, accel_b);
    result->gyro_diff  = vec3_diff_norm(gyro_a, gyro_b);

    bool cross_ok_a = dev->imu[0].online;
    bool cross_ok_b = dev->imu[1].online;

    if (dev->imu[0].online && dev->imu[1].online) {
        if (result->accel_diff > ACCEL_DIFF_THRESHOLD_G) {
            ESP_LOGW(TAG, "Accel diff too large: %.3f g", result->accel_diff);
            /* 选择加速度更接近 1g 的那路 (更可信) */
            float norm_a = fabsf(vec3_norm(accel_a) - 1.0f);
            float norm_b = fabsf(vec3_norm(accel_b) - 1.0f);
            if (norm_a > norm_b) {
                cross_ok_a = false;  /* A 偏差大, 降级 A */
            } else {
                cross_ok_b = false;  /* B 偏差大, 降级 B */
            }
        }

        if (result->gyro_diff > GYRO_DIFF_THRESHOLD_DPS) {
            ESP_LOGW(TAG, "Gyro diff too large: %.1f dps", result->gyro_diff);
            /* 两路都保留, 但降低置信度 */
        }
    }
    result->cross_check_ok = (cross_ok_a && cross_ok_b);

    /* ---- 4. 应用交叉陀螺仪偏置补偿 (在融合前!) ---- */
    if (dev->cfg.enable_cross_bias && dev->imu[0].online && dev->imu[1].online) {
        float dt = 1.0f / dev->cfg.sample_hz;
        float decay = 0.001f;
        dev->imu[0].gyro_bias_run.x += (gyro_a.x - gyro_b.x) * 0.5f * dt * decay;
        dev->imu[0].gyro_bias_run.y += (gyro_a.y - gyro_b.y) * 0.5f * dt * decay;
        dev->imu[0].gyro_bias_run.z += (gyro_a.z - gyro_b.z) * 0.5f * dt * decay;
        dev->imu[1].gyro_bias_run.x = -dev->imu[0].gyro_bias_run.x;
        dev->imu[1].gyro_bias_run.y = -dev->imu[0].gyro_bias_run.y;
        dev->imu[1].gyro_bias_run.z = -dev->imu[0].gyro_bias_run.z;
        /* 关键: 从原始数据中减去偏置, 再进入融合 */
        gyro_a.x -= dev->imu[0].gyro_bias_run.x;
        gyro_a.y -= dev->imu[0].gyro_bias_run.y;
        gyro_a.z -= dev->imu[0].gyro_bias_run.z;
        gyro_b.x -= dev->imu[1].gyro_bias_run.x;
        gyro_b.y -= dev->imu[1].gyro_bias_run.y;
        gyro_b.z -= dev->imu[1].gyro_bias_run.z;
    }

    /* ---- 5. 加权融合 ---- */
    float alpha = dev->cfg.alpha;
    icm42688_axis3f_t accel_fused, gyro_fused;

    if (cross_ok_a && cross_ok_b) {
        /* 双路正常: 加权平均 */
        accel_fused.x = alpha * accel_a.x + (1.0f - alpha) * accel_b.x;
        accel_fused.y = alpha * accel_a.y + (1.0f - alpha) * accel_b.y;
        accel_fused.z = alpha * accel_a.z + (1.0f - alpha) * accel_b.z;
        gyro_fused.x  = alpha * gyro_a.x  + (1.0f - alpha) * gyro_b.x;
        gyro_fused.y  = alpha * gyro_a.y  + (1.0f - alpha) * gyro_b.y;
        gyro_fused.z  = alpha * gyro_a.z  + (1.0f - alpha) * gyro_b.z;
    } else if (cross_ok_a) {
        /* 仅 A 有效 */
        accel_fused = accel_a;
        gyro_fused  = gyro_a;
    } else {
        /* 仅 B 有效 */
        accel_fused = accel_b;
        gyro_fused  = gyro_b;
    }

    /* ---- 6. 置信度计算 ---- */
    if (cross_ok_a && cross_ok_b) {
        /* 基于两路差异计算置信度 */
        float diff_score = 1.0f;
        diff_score -= (result->accel_diff / ACCEL_DIFF_THRESHOLD_G) * 0.3f;
        diff_score -= (result->gyro_diff  / GYRO_DIFF_THRESHOLD_DPS) * 0.2f;
        if (diff_score < 0.1f) diff_score = 0.1f;
        if (diff_score > 1.0f) diff_score = 1.0f;
        result->confidence = diff_score;
    } else if (cross_ok_a || cross_ok_b) {
        result->confidence = 0.5f;  /* 单路工作 */
    } else {
        result->confidence = 0.0f;
    }

    /* ---- 7. ESKF 融合解算 (替代旧的 Mahony) ---- */
    {
        /* 计算真实的积分步长 (秒) */
        uint64_t now_us = esp_timer_get_time();
        float dt = (now_us - dev->last_update_us) / 1000000.0f;
        if (dt <= 0.0f || dt > 0.1f) dt = 0.001f;  /* 防护: 默认 1ms */
        dev->last_update_us = now_us;

        /* 准备数据传递给 ESKF (转为 float 数组, 陀螺仪需转为 rad/s) */
        float gyro_rads[3] = {
            gyro_fused.x * (float)(M_PI / 180.0f),
            gyro_fused.y * (float)(M_PI / 180.0f),
            gyro_fused.z * (float)(M_PI / 180.0f)
        };
        float accel_g[3] = { accel_fused.x, accel_fused.y, accel_fused.z };

        /* 1. 预测步 (高频执行, 在 IRAM 中运行) */
        eskf_predict(&dev->eskf_fused, &dev->eskf_state_fused, gyro_rads, dt);

        /* 2. 更新步 (利用加速度校正, 在 IRAM 中运行) */
        eskf_update_accel(&dev->eskf_fused, &dev->eskf_state_fused, accel_g);
    }

    /* ---- 8. 输出 ---- */
    /* 从 ESKF 标称状态提取四元数 */
    result->quat.w = dev->eskf_state_fused.q[0];
    result->quat.x = dev->eskf_state_fused.q[1];
    result->quat.y = dev->eskf_state_fused.q[2];
    result->quat.z = dev->eskf_state_fused.q[3];

    result->accel = accel_fused;
    result->gyro  = gyro_fused;
    result->timestamp_us = esp_timer_get_time();

    dev->fused_count++;

    return ICM42688_OK;
}

/* ============================================================
 *  API — 静态校准
 * ============================================================ */
icm42688_err_t dual_imu_calibrate(dual_imu_dev_t *dev, uint32_t samples)
{
    if (!dev || !dev->initialized) return ICM42688_ERR_NOT_INIT;
    if (samples == 0) samples = 300;

    ESP_LOGI(TAG, "=== 双 IMU 交叉校准 (%lu 样本) ===", (unsigned long)samples);

    /* 分别校准两路偏移 */
    icm42688_err_t err;
    err = icm42688_calibrate_bias(dev->imu[0].dev, samples);
    if (err != ICM42688_OK) {
        ESP_LOGW(TAG, "IMU-A 校准失败");
    }

    err = icm42688_calibrate_bias(dev->imu[1].dev, samples);
    if (err != ICM42688_OK) {
        ESP_LOGW(TAG, "IMU-B 校准失败");
    }

    /* 如果未指定安装偏差, 尝试自动估算 */
    if (dev->cfg.enable_misalign &&
        dev->cfg.misalign_roll == 0 && dev->cfg.misalign_pitch == 0 && dev->cfg.misalign_yaw == 0) {

        ESP_LOGI(TAG, "自动检测安装偏差...");

        /* 采集静止数据, 比较两路加速度方向差异来估算安装角 */
        icm42688_axis3f_t sum_a = {0}, sum_b = {0};
        for (uint32_t i = 0; i < samples; i++) {
            icm42688_reading_t r_a, r_b;
            icm42688_read_polling(dev->imu[0].dev, &r_a);
            icm42688_read_polling(dev->imu[1].dev, &r_b);
            sum_a.x += r_a.accel_g.x;
            sum_a.y += r_a.accel_g.y;
            sum_a.z += r_a.accel_g.z;
            sum_b.x += r_b.accel_g.x;
            sum_b.y += r_b.accel_g.y;
            sum_b.z += r_b.accel_g.z;
            vTaskDelay(pdMS_TO_TICKS(1));
        }

        /* 归一化平均加速度向量 */
        float inv = 1.0f / (float)samples;
        icm42688_axis3f_t na = { sum_a.x*inv, sum_a.y*inv, sum_a.z*inv };
        icm42688_axis3f_t nb = { sum_b.x*inv, sum_b.y*inv, sum_b.z*inv };

        float la = vec3_norm(na);
        float lb = vec3_norm(nb);
        if (la > 0.01f) { na.x/=la; na.y/=la; na.z/=la; }
        if (lb > 0.01f) { nb.x/=lb; nb.y/=lb; nb.z/=lb; }

        /*
         * 从两个归一化重力向量估算旋转矩阵:
         * R_align ≈ R_from_a_to_b
         * 使用 Rodrigues 公式的简化:
         *   cross = na × nb
         *   dot   = na · nb
         *   angle = atan2(|cross|, dot)
         */
        float cx = na.y*nb.z - na.z*nb.y;
        float cy = na.z*nb.x - na.x*nb.z;
        float cz = na.x*nb.y - na.y*nb.x;
        float dot = na.x*nb.x + na.y*nb.y + na.z*nb.z;
        float angle = atan2f(sqrtf(cx*cx + cy*cy + cz*cz), dot);

        if (fabsf(angle) > 0.01f) {
            /* 归一化叉积 = 旋转轴 */
            float inv_cross = 1.0f / sqrtf(cx*cx + cy*cy + cz*cz);
            float nx = cx * inv_cross;
            float ny = cy * inv_cross;
            float nz = cz * inv_cross;

            /* 罗德里格斯旋转公式 (Rodrigues' rotation formula):
             *   R = I + sin(θ)*[k]× + (1-cos(θ))*[k]×²
             * 直接生成 R_align, 不经过欧拉角 */
            float s = sinf(angle);
            float c = cosf(angle);
            float t = 1.0f - c;

            dev->R_align.m[0][0] = t*nx*nx + c;
            dev->R_align.m[0][1] = t*nx*ny - s*nz;
            dev->R_align.m[0][2] = t*nx*nz + s*ny;
            dev->R_align.m[1][0] = t*nx*ny + s*nz;
            dev->R_align.m[1][1] = t*ny*ny + c;
            dev->R_align.m[1][2] = t*ny*nz - s*nx;
            dev->R_align.m[2][0] = t*nx*nz - s*ny;
            dev->R_align.m[2][1] = t*ny*nz + s*nx;
            dev->R_align.m[2][2] = t*nz*nz + c;

            ESP_LOGI(TAG, "自动检测安装偏差 (Rodrigues): axis=[%.4f,%.4f,%.4f] angle=%.2f°",
                     nx, ny, nz, angle * 180.0f / M_PI);
        } else {
            ESP_LOGI(TAG, "两路传感器几乎对齐, 无需补偿");
        }
    }

    /* 验证校准效果: 再读一组, 计算差异 */
    icm42688_axis3f_t diff_sum = {0};
    uint32_t verify_count = 50;
    for (uint32_t i = 0; i < verify_count; i++) {
        icm42688_reading_t r_a, r_b;
        icm42688_read_polling(dev->imu[0].dev, &r_a);
        icm42688_read_polling(dev->imu[1].dev, &r_b);

        icm42688_axis3f_t b_comp = mat3_mul_vec(dev->R_align, r_b.accel_g);
        float dx = r_a.accel_g.x - b_comp.x;
        float dy = r_a.accel_g.y - b_comp.y;
        float dz = r_a.accel_g.z - b_comp.z;
        diff_sum.x += dx*dx;
        diff_sum.y += dy*dy;
        diff_sum.z += dz*dz;
        vTaskDelay(pdMS_TO_TICKS(1));
    }

    float rmse = sqrtf((diff_sum.x + diff_sum.y + diff_sum.z) / verify_count);
    ESP_LOGI(TAG, "校准后加速度 RMSE: %.4f g", rmse);
    ESP_LOGI(TAG, "=== 校准完成 ===");

    return ICM42688_OK;
}

/* ============================================================
 *  API — 其他
 * ============================================================ */
void dual_imu_set_misalignment(dual_imu_dev_t *dev,
                                float roll_deg, float pitch_deg, float yaw_deg)
{
    if (!dev) return;
    dev->cfg.misalign_roll  = roll_deg;
    dev->cfg.misalign_pitch = pitch_deg;
    dev->cfg.misalign_yaw   = yaw_deg;
    dev->R_align = mat3_from_euler(roll_deg, pitch_deg, yaw_deg);
}

float dual_imu_get_confidence(const dual_imu_dev_t *dev)
{
    /* 从最近一次结果获取 (简化: 重新计算) */
    if (!dev || !dev->initialized) return 0.0f;
    bool a = dev->imu[0].online;
    bool b = dev->imu[1].online;
    if (a && b) return 1.0f;
    if (a || b) return 0.5f;
    return 0.0f;
}

const dual_imu_unit_t *dual_imu_get_unit(const dual_imu_dev_t *dev, int idx)
{
    if (!dev || idx < 0 || idx > 1) return NULL;
    return &dev->imu[idx];
}

void dual_imu_reset(dual_imu_dev_t *dev)
{
    if (!dev) return;
    /* 重新初始化 ESKF */
    eskf_init(&dev->eskf_fused, &dev->eskf_state_fused);
    dev->last_update_us = esp_timer_get_time();
    dev->fused_count = 0;
    dev->imu[0].gyro_bias_run = (icm42688_axis3f_t){0};
    dev->imu[1].gyro_bias_run = (icm42688_axis3f_t){0};
    ESP_LOGI(TAG, "Dual IMU reset (ESKF reinitialized)");
}
