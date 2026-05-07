/**
 * @file main.c
 * @brief 双 ICM-42688-P 融合解算 + ESP-NOW 广播
 *
 * 硬件连接:
 *   IMU-A (SPI2):  SCLK=48 MOSI=47 MISO=21 CS=45 INT1=14 LDO_EN1=4
 *   IMU-B (SPI3):  SCLK=41 MOSI=40 MISO=39 CS=42 INT1=38 LDO_EN2=5
 *
 * 功能:
 *   1. 双路独立 SPI 并发读取
 *   2. 交叉校准 (自动检测安装偏差角)
 *   3. 6-ESKF 融合解算
 *   4. 异常值检测 & 单路降级
 *   5. ESP-NOW 广播聚合数据
 *
 * 优化:
 *   A. LDO 强控上电时序 (50ms 延迟, 防错过首个中断)
 *   B. 双路独立 SPI2/SPI3 (绝对并发, 无总线竞争)
 *   C. 中断驱动读取 (INT1 → GPIO 中断 → 信号量)
 *   D. 零 malloc DMA 缓冲 (init 时预分配 32B)
 *   E. 10帧聚合发送 (1000Hz采样 → 100Hz发送)
 *   F. 防死锁异步 ESP-NOW 收发
 */

/* ============================================================
 *  虚拟数据模式 (Mock Mode)
 *  置1: 跳过真实 IMU 初始化, 用正弦波模拟数据, 验证 ESKF/聚合/发送链路
 *  置0: 正常硬件驱动模式
 * ============================================================ */
#define CONFIG_IMU_MOCK_MODE  1

#include <stdio.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"

#include "icm42688.h"
#include "icm42688_alg.h"
#include "icm42688_dual.h"
#include "hardcore_eskf.h"
#include "net_send.h"
#include "time_sync.h"
#include "esp_wifi.h"

static const char *TAG = "main";

/* ============================================================
 *  LDO 电源使能引脚
 * ============================================================ */
#define PIN_LDO_EN1   4      /* IMU-A LDO 使能 */
#define PIN_LDO_EN2   5      /* IMU-B LDO 使能 */

/* ============================================================
 *  IMU-A 引脚 (SPI2)
 * ============================================================ */
#define PIN_SCLK_A    13
#define PIN_MOSI_A    12
#define PIN_MISO_A    11
#define PIN_CS_A      14
#define PIN_INT_A     10
#define SPI_HOST_A    SPI2_HOST

/* ============================================================
 *  IMU-B 引脚 (SPI3)
 * ============================================================ */
#define PIN_SCLK_B    17
#define PIN_MOSI_B    16
#define PIN_MISO_B    15
#define PIN_CS_B      18
#define PIN_INT_B     7
#define SPI_HOST_B    SPI3_HOST

/* 节点 ID: 启动时从 MAC 地址低位自动推演, 无需硬编码 */
static uint8_t s_node_id = 0;

/* ============================================================
 *  传感器参数
 * ============================================================ */
#define ACCEL_FS    ICM42688_ACCEL_4G
#define GYRO_FS     ICM42688_GYRO_2000DPS
#define SAMPLE_HZ   1000   /* IMU 采样率 */
#define OUTPUT_HZ   100    /* 数据上报频率 */

/* ============================================================
 *  融合参数
 * ============================================================ */
#define FUSION_ALPHA       0.5f   /* 融合权重: 0.5 = 等权 */
#define MISALIGN_ROLL      0.0f   /* IMU-B 相对 A 的安装偏差 (度), 校准后自动更新 */
#define MISALIGN_PITCH     0.0f
#define MISALIGN_YAW       0.0f

/* ============================================================
 *  ESP-NOW 配置 (无需路由器, 底层广播)
 * ============================================================ */

/* ============================================================
 *  app_main
 * ============================================================ */
void app_main(void)
{
    ESP_LOGI(TAG, "=== 双 ICM-42688-P + ESP-NOW ===");

#if CONFIG_IMU_MOCK_MODE
    ESP_LOGW(TAG, "*** MOCK MODE ENABLED — 无真实 IMU, 使用模拟数据 ***");
#endif

    /* ================================================================
     *  LDO 强控上电时序
     * ================================================================ */
#if !CONFIG_IMU_MOCK_MODE
    gpio_config_t ldo_io = {
        .pin_bit_mask = (1ULL << PIN_LDO_EN1) | (1ULL << PIN_LDO_EN2),
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    gpio_config(&ldo_io);
    gpio_set_level(PIN_LDO_EN1, 1);
    gpio_set_level(PIN_LDO_EN2, 1);
    ESP_LOGI(TAG, "LDO EN1(IO%d) + EN2(IO%d) 拉高, 等待 50ms 稳压...", PIN_LDO_EN1, PIN_LDO_EN2);
    vTaskDelay(pdMS_TO_TICKS(50));
    ESP_LOGI(TAG, "LDO 稳压完成, 开始 SPI 初始化");
#endif

    icm42688_err_t err;

#if !CONFIG_IMU_MOCK_MODE
    /* ---- 1. IMU-A SPI 配置 (独立 SPI2) ---- */
    icm42688_spi_cfg_t spi_cfg_a = {
        .spi_host       = SPI_HOST_A,
        .pin_sclk       = PIN_SCLK_A,
        .pin_mosi       = PIN_MOSI_A,
        .pin_miso       = PIN_MISO_A,
        .pin_cs         = PIN_CS_A,
        .pin_int        = PIN_INT_A,
        .clock_speed_hz = 10000000,
    };

    /* ---- 2. IMU-B SPI 配置 (独立 SPI3) ---- */
    icm42688_spi_cfg_t spi_cfg_b = {
        .spi_host       = SPI_HOST_B,
        .pin_sclk       = PIN_SCLK_B,
        .pin_mosi       = PIN_MOSI_B,
        .pin_miso       = PIN_MISO_B,
        .pin_cs         = PIN_CS_B,
        .pin_int        = PIN_INT_B,
        .clock_speed_hz = 10000000,
    };

    /* ---- 3. 传感器配置 (两路相同) ---- */
    icm42688_cfg_t sensor_cfg = {
        .accel_fs    = ACCEL_FS,
        .gyro_fs     = GYRO_FS,
        .accel_odr   = ICM42688_ODR_1000HZ,
        .gyro_odr    = ICM42688_ODR_1000HZ,
        .enable_fifo = false,
    };

    /* ---- 4. 初始化 IMU-A ---- */
    ESP_LOGI(TAG, "初始化 IMU-A...");
    icm42688_dev_t imu_a;
    icm42688_err_t err = icm42688_init(&imu_a, &spi_cfg_a, &sensor_cfg);
    if (err != ICM42688_OK) {
        ESP_LOGE(TAG, "IMU-A init failed: %d", err);
        return;
    }

    /* ---- 5. 初始化 IMU-B ---- */
    ESP_LOGI(TAG, "初始化 IMU-B...");
    icm42688_dev_t imu_b;
    err = icm42688_init(&imu_b, &spi_cfg_b, &sensor_cfg);
    if (err != ICM42688_OK) {
        ESP_LOGE(TAG, "IMU-B init failed: %d", err);
        return;
    }

    /* ---- 5.1 配置中断驱动 (Data Ready → GPIO 中断 → 信号量) ---- */
    ESP_LOGI(TAG, "配置中断驱动读取...");
    err = icm42688_init_interrupt(&imu_a, PIN_INT_A);
    if (err != ICM42688_OK) {
        ESP_LOGW(TAG, "IMU-A 中断配置失败, 降级为轮询模式");
    }
    err = icm42688_init_interrupt(&imu_b, PIN_INT_B);
    if (err != ICM42688_OK) {
        ESP_LOGW(TAG, "IMU-B 中断配置失败, 降级为轮询模式");
    }

    /* ---- 6. 双 IMU 交叉校准 ---- */
    ESP_LOGI(TAG, "请保持传感器静止, 开始交叉校准...");
    vTaskDelay(pdMS_TO_TICKS(1000));
#else
    /* ---- MOCK MODE: 跳过硬件初始化, 手动构造虚拟设备 ---- */
    icm42688_dev_t imu_a = {0};
    icm42688_dev_t imu_b = {0};
    imu_a.initialized = true;
    imu_b.initialized = true;
    ESP_LOGI(TAG, "Mock: 虚拟 IMU-A/B 已就绪, 跳过校准");
#endif

    dual_imu_cfg_t dual_cfg = {
        .alpha             = FUSION_ALPHA,
        .kp                = 1.0f,
        .ki                = 0.005f,
        .sample_hz         = SAMPLE_HZ,
        .enable_misalign   = true,
        .enable_cross_bias = true,
        .misalign_roll     = MISALIGN_ROLL,
        .misalign_pitch    = MISALIGN_PITCH,
        .misalign_yaw      = MISALIGN_YAW,
    };

    dual_imu_dev_t dual_dev;
    err = dual_imu_init(&dual_dev, &imu_a, &imu_b, &dual_cfg);
    if (err != ICM42688_OK) {
        ESP_LOGE(TAG, "Dual IMU init failed: %d", err);
        return;
    }

    /* 执行交叉校准 (自动检测安装偏差) */
    err = dual_imu_calibrate(&dual_dev, 500);
    if (err != ICM42688_OK) {
        ESP_LOGW(TAG, "Cross calibration failed, continuing");
    }

    /* ---- 7. 初始化 ESP-NOW 广播 + 时间同步 ---- */
    ESP_LOGI(TAG, "初始化 ESP-NOW 广播 (无需路由器)");
    if (!net_wifi_init(NULL)) {
        ESP_LOGE(TAG, "ESP-NOW WiFi init 失败, 仅串口输出");
    }
    if (!net_udp_init(NULL)) {
        ESP_LOGE(TAG, "ESP-NOW init 失败, 仅串口输出");
    }

    /* 动态 MAC 衍生 Node ID: 取 MAC 末字节, 保证 12 节点唯一 */
    uint8_t mac[6];
    esp_wifi_get_mac(WIFI_IF_STA, mac);
    s_node_id = mac[5];  /* MAC 最后一字节作为 Node ID */
    net_set_node_id(s_node_id);
    net_time_sync_init();
    ESP_LOGI(TAG, "ESP-NOW 就绪 (MAC:%02X:%02X:%02X:%02X:%02X:%02X → node_id=0x%02X)",
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5], s_node_id);

    /* ---- 8. 主循环 (1000Hz 采样, 5帧聚合 → 200Hz 发送) ---- */
    dual_imu_result_t result;
    uint32_t pkt_count = 0;
    uint32_t send_count = 0;

    /* 聚合缓冲: 攒满 5 帧 (5ms) 再发一次, 248B < 250B ESP-NOW 上限 */
    net_aggregated_packet_t agg_pkt = {
        .magic = {'I', 'M', 'U', 'A'},
        .node_id = s_node_id,
    };

    ESP_LOGI(TAG, "=== 极速四元数模式 (1000Hz采样, 5帧聚合→200Hz发送) ===");

#if CONFIG_IMU_MOCK_MODE
    ESP_LOGW(TAG, "*** MOCK MODE: 正弦波模拟数据, 验证 ESKF/聚合/发送链路 ***");
    /* 确保 ESKF 初始四元数为单位四元数 [1,0,0,0] */
    dual_dev.eskf_state_fused.q[0] = 1.0f;
    dual_dev.eskf_state_fused.q[1] = 0.0f;
    dual_dev.eskf_state_fused.q[2] = 0.0f;
    dual_dev.eskf_state_fused.q[3] = 0.0f;
#endif

    uint32_t mock_tick = 0;
    int log_div = 0;  /* 限流打印计数器 */

    while (1) {
#if !CONFIG_IMU_MOCK_MODE
        /* ---- 严格双硬件中断同步等待 Data Ready ---- */
        icm42688_err_t drdy_a = icm42688_wait_drdy(&imu_a, 2);
        icm42688_err_t drdy_b = icm42688_wait_drdy(&imu_b, 2);

        if (drdy_a == ICM42688_ERR_TIMEOUT && drdy_b == ICM42688_ERR_TIMEOUT) {
            vTaskDelay(pdMS_TO_TICKS(1));
            continue;
        }

        /* 双路同步读取 + 融合 */
        err = dual_imu_update(&dual_dev, &result);
        if (err != ICM42688_OK) {
            vTaskDelay(pdMS_TO_TICKS(1));
            continue;
        }
#else
        /* ---- MOCK MODE: 生成模拟 3D 运动数据 ---- */
        {
            float t = mock_tick * 0.001f;  /* 时间 (秒), 1kHz */

            /* 加速度: 1g 重力分量在 XY 平面随时间缓慢转动 */
            float gravity_angle = t * 0.5f;  /* 0.5 rad/s 旋转 */
            float ax = sinf(gravity_angle);
            float ay = cosf(gravity_angle);
            float az = 0.1f;

            /* NaN 保护: 确保加速度向量归一化后有效 */
            float a_norm = sqrtf(ax*ax + ay*ay + az*az);
            if (a_norm < 0.01f) { ax = 0.0f; ay = 0.0f; az = 1.0f; a_norm = 1.0f; }
            result.accel.x = ax / a_norm;
            result.accel.y = ay / a_norm;
            result.accel.z = az / a_norm;

            /* 陀螺仪: Z轴 50dps 旋转 + X轴 10dps 小幅漂移 */
            result.gyro.x = 10.0f + 2.0f * sinf(t * 0.3f);  /* X: 10dps + 漂移 */
            result.gyro.y = 3.0f * cosf(t * 0.7f);          /* Y: 微小摆动 */
            result.gyro.z = 50.0f;                           /* Z: 恒定 50dps 旋转 */

            /* 直接喂给 ESKF 解算 */
            {
                float gyro_rads[3] = {
                    result.gyro.x * (float)(M_PI / 180.0f),
                    result.gyro.y * (float)(M_PI / 180.0f),
                    result.gyro.z * (float)(M_PI / 180.0f)
                };
                float accel_g[3] = { result.accel.x, result.accel.y, result.accel.z };

                eskf_predict(&dual_dev.eskf_fused, &dual_dev.eskf_state_fused, gyro_rads, 0.001f);
                eskf_update_accel(&dual_dev.eskf_fused, &dual_dev.eskf_state_fused, accel_g);

                /* 从 ESKF 提取四元数 */
                result.quat.w = dual_dev.eskf_state_fused.q[0];
                result.quat.x = dual_dev.eskf_state_fused.q[1];
                result.quat.y = dual_dev.eskf_state_fused.q[2];
                result.quat.z = dual_dev.eskf_state_fused.q[3];

                /* 四元数 NaN 保护: 如果结果异常则重置为单位四元数 */
                float qn = result.quat.w*result.quat.w + result.quat.x*result.quat.x
                         + result.quat.y*result.quat.y + result.quat.z*result.quat.z;
                if (isnan(qn) || qn < 0.5f || qn > 2.0f) {
                    ESP_LOGW(TAG, "Mock: ESKF 四元数异常 (qn=%.4f), 重置", qn);
                    dual_dev.eskf_state_fused.q[0] = 1.0f;
                    dual_dev.eskf_state_fused.q[1] = 0.0f;
                    dual_dev.eskf_state_fused.q[2] = 0.0f;
                    dual_dev.eskf_state_fused.q[3] = 0.0f;
                    result.quat = (quat_t){1, 0, 0, 0};
                }

                result.confidence = 1.0f;
                result.accel_diff = 0.0f;
                result.gyro_diff = 0.0f;
                result.cross_check_ok = true;
            }

            mock_tick++;
        }
#endif

        /* ---- 聚合: 每帧数据存入缓冲 ---- */
        int slot = agg_pkt.frame_count;
        if (slot < NET_AGGREGATE_FRAMES) {
            net_frame_t *f = &agg_pkt.frames[slot];
            f->accel[0] = result.accel.x;
            f->accel[1] = result.accel.y;
            f->accel[2] = result.accel.z;
            f->gyro[0]  = result.gyro.x;
            f->gyro[1]  = result.gyro.y;
            f->gyro[2]  = result.gyro.z;
            f->quat[0]  = result.quat.w;
            f->quat[1]  = result.quat.x;
            f->quat[2]  = result.quat.y;
            f->quat[3]  = result.quat.z;
            f->timestamp_us = (uint64_t)net_get_synced_time();
            agg_pkt.frame_count = slot + 1;
        }

        /* ---- 攒满 10 帧 (10ms) → 聚合发送 ---- */
        if (agg_pkt.frame_count >= NET_AGGREGATE_FRAMES) {
            bool send_ok = net_udp_send_aggregated(&agg_pkt);
            agg_pkt.frame_count = 0;  /* 重置缓冲 */
            send_count++;

            if (send_count % 20 == 0) {
                ESP_LOGI(TAG, "[%lu] QW=%.4f Conf=%.0f%% | %s | TX:%lu",
                         (unsigned long)send_count,
                         result.quat.w,
                         result.confidence * 100.0f,
                         send_ok ? "TX✓" : "TX✗",
                         (unsigned long)net_espnow_get_send_ok());

                const char *sta = dual_dev.imu[0].online ? "ON" : "OFF";
                const char *stb = dual_dev.imu[1].online ? "ON" : "OFF";
                ESP_LOGI(TAG, "  IMU-A: %s (IRQ:%lu) | IMU-B: %s (IRQ:%lu) | Sync:%s #%lu",
                         sta, (unsigned long)icm42688_get_int_count(&imu_a),
                         stb, (unsigned long)icm42688_get_int_count(&imu_b),
                         net_time_sync_valid() ? "OK" : "WAIT",
                         (unsigned long)net_time_sync_count());
            }
        }

        pkt_count++;

        /* ---- 限流日志: Mock 模式每 100 次 (~10Hz) 打印一次解算状态 ---- */
#if CONFIG_IMU_MOCK_MODE
        if (++log_div >= 100) {
            ESP_LOGI(TAG, "[MOCK] Quat: W:%.3f X:%.3f Y:%.3f Z:%.3f | TX_Cnt:%lu | T:%.1fs",
                     result.quat.w, result.quat.x, result.quat.y, result.quat.z,
                     (unsigned long)net_espnow_get_send_ok(),
                     mock_tick * 0.001f);
            log_div = 0;
        }
#endif

        /* ---- 强制让出 CPU 给 IDLE 任务 (防止 Task WDT) ---- */
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}
