/**
 * @file main.c
 * @brief 双 ICM-42688-P 融合解算 + WiFi 数据上报
 *
 * 硬件连接 (ESP32-S3 常见引脚, 可自行修改):
 *
 *   两路 IMU 共享 SPI2 总线:
 *     SCLK → GPIO 12
 *     MOSI → GPIO 11
 *     MISO → GPIO 13
 *     CS_A → GPIO 10
 *     CS_B → GPIO 34
 *   
 * 功能:
 *   1. 双 IMU 同步读取
 *   2. 交叉校准 (自动检测安装偏差角)
 *   3. 加权互补滤波融合
 *   4. 异常值检测 & 单路降级
 *   5. WiFi UDP 实时发送融合结果
 *
 * 优化:
 *   A. 中断驱动读取 (INT1 → GPIO 中断 → 信号量唤醒任务)
 *   B. IRAM 优化 (sdkconfig.defaults)
 *   C. 全局时间戳同步 (主机广播 offset, 节点校正) *  D. SPI 共享总线 (统一 SPI2_HOST, 不同 CS)
 *  E. 零 malloc DMA 缓冲 (init 时预分配 32B)
 *  F. 10帧聚合发送 (1000Hz采样 → 100Hz发送) */

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
#include "time_sync.h"

static const char *TAG = "main";

/* ============================================================
 *  SPI 共享总线引脚 (统一使用 SPI2_HOST)
 * ============================================================ */
#define PIN_SCLK      12
#define PIN_MOSI      11
#define PIN_MISO      13
#define PIN_CS_A      10
#define PIN_CS_B      34
#define SPI_HOST      SPI2_HOST

/* ============================================================
 *  中断引脚
 * ============================================================ */
#define PIN_INT_A     46     /* IMU-A INT1 引脚 */
#define PIN_INT_B     9      /* IMU-B INT1 引脚 */

/* ============================================================
 *  节点 ID (用于时间同步, 每个节点唯一)
 * ============================================================ */
#define NODE_ID       0x01

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

    /* ---- 1. IMU-A SPI 配置 (共享总线, 仅 CS 不同) ---- */
    icm42688_spi_cfg_t spi_cfg_a = {
        .spi_host       = SPI_HOST,
        .pin_sclk       = PIN_SCLK,
        .pin_mosi       = PIN_MOSI,
        .pin_miso       = PIN_MISO,
        .pin_cs         = PIN_CS_A,
        .pin_int        = PIN_INT_A,
        .clock_speed_hz = 10000000,
    };

    /* ---- 2. IMU-B SPI 配置 (共用总线, 不同 CS) ---- */
    icm42688_spi_cfg_t spi_cfg_b = {
        .spi_host       = SPI_HOST,  /* 同一 SPI 主机 */
        .pin_sclk       = PIN_SCLK,  /* 共享引脚 */
        .pin_mosi       = PIN_MOSI,
        .pin_miso       = PIN_MISO,
        .pin_cs         = PIN_CS_B,  /* 不同 CS */
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
    net_set_node_id(NODE_ID);
    net_time_sync_init();
    ESP_LOGI(TAG, "ESP-NOW + 时间同步就绪 (node_id=0x%02X)", NODE_ID);

    /* ---- 8. 主循环 (1000Hz 采样, 10帧聚合 → 100Hz 发送) ---- */
    dual_imu_result_t result;
    uint32_t pkt_count = 0;
    uint32_t send_count = 0;

    /* 聚合缓冲: 攒满 10 帧再发一次 */
    net_aggregated_packet_t agg_pkt = {
        .magic = {'I', 'M', 'U', 'A'},
        .node_id = NODE_ID,
    };

    ESP_LOGI(TAG, "=== 开始双路融合解算 (1000Hz采样, 100Hz聚合发送) ===");

    while (1) {
        /* ---- 中断驱动等待 Data Ready ---- */
        icm42688_err_t drdy_err = icm42688_wait_drdy(&imu_a, 2);
        if (drdy_err == ICM42688_ERR_TIMEOUT) {
            vTaskDelay(pdMS_TO_TICKS(1));
        }

        /* 双路同步读取 + 融合 */
        err = dual_imu_update(&dual_dev, &result);
        if (err != ICM42688_OK) {
            vTaskDelay(pdMS_TO_TICKS(1));
            continue;
        }

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
            f->euler[0] = result.euler.roll;
            f->euler[1] = result.euler.pitch;
            f->euler[2] = result.euler.yaw;
            f->temp     = result.temperature;
            f->timestamp_us = (uint64_t)net_get_synced_time();
            agg_pkt.frame_count = slot + 1;
        }

        /* ---- 攒满 10 帧 (10ms) → 聚合发送 ---- */
        if (agg_pkt.frame_count >= NET_AGGREGATE_FRAMES) {
            bool send_ok = net_udp_send_aggregated(&agg_pkt);
            agg_pkt.frame_count = 0;  /* 重置缓冲 */
            send_count++;

            if (send_count % 10 == 0) {
                ESP_LOGI(TAG, "[%lu] R=%5.1f° P=%5.1f° Y=%6.1f° | "
                         "Conf=%.0f%% | %s | TX:%lu",
                         (unsigned long)send_count,
                         result.euler.roll, result.euler.pitch, result.euler.yaw,
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
    }
}
