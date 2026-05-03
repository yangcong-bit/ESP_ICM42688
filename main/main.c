/**
 * @file main.c
 * @brief 双 ICM-42688-P 融合解算 + WiFi 数据上报
 *
 * 硬件连接 (ESP32-S3 常见引脚, 可自行修改):
 *
 *   IMU-A (SPI2):
 *     SCLK → GPIO 12
 *     MOSI → GPIO 11
 *     MISO → GPIO 13
 *     CS_A → GPIO 10
 *
 *   IMU-B (SPI3, 或与 SPI2 共享总线用不同 CS):
 *     SCLK → GPIO 36
 *     MOSI → GPIO 35
 *     MISO → GPIO 37
 *     CS_B → GPIO 34
 *
 *   如果两路 SPI 共享总线, 可以共用 SCLK/MOSI/MISO, 只需不同 CS
 *   
 * 功能:
 *   1. 双 IMU 同步读取
 *   2. 交叉校准 (自动检测安装偏差角)
 *   3. 加权互补滤波融合
 *   4. 异常值检测 & 单路降级
 *   5. WiFi UDP 实时发送融合结果
 */

#include <stdio.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"

#include "icm42688.h"
#include "icm42688_alg.h"
#include "icm42688_dual.h"
#include "net_send.h"

static const char *TAG = "main";

/* ============================================================
 *  IMU-A 引脚 (SPI2)
 * ============================================================ */
#define PIN_SCLK_A    12
#define PIN_MOSI_A    11
#define PIN_MISO_A    13
#define PIN_CS_A      10
#define SPI_HOST_A    SPI2_HOST

/* ============================================================
 *  IMU-B 引脚 (SPI3)
 * ============================================================ */
#define PIN_SCLK_B    36
#define PIN_MOSI_B    35
#define PIN_MISO_B    37
#define PIN_CS_B      34
#define SPI_HOST_B    SPI3_HOST

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
    ESP_LOGI(TAG, "=== 双 ICM-42688-P AHRS + ESP-NOW Demo ===");

    /* ---- 1. IMU-A SPI 配置 ---- */
    icm42688_spi_cfg_t spi_cfg_a = {
        .spi_host       = SPI_HOST_A,
        .pin_sclk       = PIN_SCLK_A,
        .pin_mosi       = PIN_MOSI_A,
        .pin_miso       = PIN_MISO_A,
        .pin_cs         = PIN_CS_A,
        .clock_speed_hz = 10000000,
    };

    /* ---- 2. IMU-B SPI 配置 ---- */
    icm42688_spi_cfg_t spi_cfg_b = {
        .spi_host       = SPI_HOST_B,
        .pin_sclk       = PIN_SCLK_B,
        .pin_mosi       = PIN_MOSI_B,
        .pin_miso       = PIN_MISO_B,
        .pin_cs         = PIN_CS_B,
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

    /* ---- 6. 双 IMU 交叉校准 ---- */
    ESP_LOGI(TAG, "请保持传感器静止, 开始交叉校准...");
    vTaskDelay(pdMS_TO_TICKS(1000));

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

    /* ---- 7. 初始化 ESP-NOW 广播 ---- */
    ESP_LOGI(TAG, "初始化 ESP-NOW 广播 (无需路由器)");
    if (!net_wifi_init(NULL)) {
        ESP_LOGE(TAG, "ESP-NOW WiFi init 失败, 仅串口输出");
    }
    if (!net_udp_init(NULL)) {
        ESP_LOGE(TAG, "ESP-NOW init 失败, 仅串口输出");
    }
    ESP_LOGI(TAG, "ESP-NOW 广播就绪, 所有同信道 ESP32 均可接收");

    /* ---- 8. 主循环 ---- */
    dual_imu_result_t result;
    int64_t last_output = esp_timer_get_time();
    const int64_t output_interval_us = 1000000 / OUTPUT_HZ;
    uint32_t pkt_count = 0;

    ESP_LOGI(TAG, "=== 开始双路融合解算 (%dHz) ===", OUTPUT_HZ);

    while (1) {
        /* 双路同步读取 + 融合 */
        err = dual_imu_update(&dual_dev, &result);
        if (err != ICM42688_OK) {
            vTaskDelay(pdMS_TO_TICKS(5));
            continue;
        }

        /* 控制上报频率 */
        int64_t now = esp_timer_get_time();
        if ((now - last_output) >= output_interval_us) {
            last_output = now;

            /* 构建 ESP-NOW 发送数据包 */
            net_imu_packet_t pkt = {
                .accel = {result.accel.x, result.accel.y, result.accel.z},
                .gyro  = {result.gyro.x,  result.gyro.y,  result.gyro.z},
                .temp  = result.temperature,
                .quat  = {result.quat.w, result.quat.x, result.quat.y, result.quat.z},
                .euler = {result.euler.roll, result.euler.pitch, result.euler.yaw},
                .timestamp_us = result.timestamp_us,
            };

            /* ESP-NOW 广播发送 */
            bool send_ok = net_udp_send_imu(&pkt);

            pkt_count++;

            /* 每 20 包打印一次详细信息 */
            if (pkt_count % 20 == 0) {
                ESP_LOGI(TAG, "[%lu] R=%5.1f° P=%5.1f° Y=%6.1f° | "
                         "Conf=%.0f%% | ΔA=%.3fg ΔG=%.1fdps | %s",
                         (unsigned long)pkt_count,
                         result.euler.roll, result.euler.pitch, result.euler.yaw,
                         result.confidence * 100.0f,
                         result.accel_diff, result.gyro_diff,
                         send_ok ? "TX✓" : "TX✗");

                /* 显示两路 IMU 在线状态 */
                const char *sta = dual_dev.imu[0].online ? "ON" : "OFF";
                const char *stb = dual_dev.imu[1].online ? "ON" : "OFF";
                ESP_LOGI(TAG, "  IMU-A: %s | IMU-B: %s | Cross: %s",
                         sta, stb, result.cross_check_ok ? "OK" : "MISMATCH");
            }
        }

        vTaskDelay(pdMS_TO_TICKS(1));
    }
}
