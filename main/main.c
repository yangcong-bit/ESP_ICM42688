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
#define PIN_SCLK_A    48
#define PIN_MOSI_A    47
#define PIN_MISO_A    21
#define PIN_CS_A      45
#define PIN_INT_A     14
#define SPI_HOST_A    SPI2_HOST

/* ============================================================
 *  IMU-B 引脚 (SPI3)
 * ============================================================ */
#define PIN_SCLK_B    41
#define PIN_MOSI_B    40
#define PIN_MISO_B    39
#define PIN_CS_B      42
#define PIN_INT_B     38
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
/* ============================================================
 *  IMU 采集任务 (优先级 15, Core 1, 绝对最高路权)
 *  底层无线发送不能打断传感器读取
 * ============================================================ */
static void imu_task(void *arg)
{
    dual_imu_dev_t *dual_dev = (dual_imu_dev_t *)arg;
    dual_imu_result_t result;

    ESP_LOGI(TAG, "imu_task 启动 (Core1, 优先级 15)");

    while (1) {
        /* 双路 EventGroup 并发等待: 任一路触发即进入读取 */
        icm42688_err_t drdy = icm42688_wait_drdy_group(
            &dual_dev->imu[0].dev[0], &dual_dev->imu[1].dev[0], 2);
        if (drdy == ICM42688_ERR_TIMEOUT) {
            vTaskDelay(pdMS_TO_TICKS(1));
            continue;
        }

        /* 双路同步读取 + 融合 */
        icm42688_err_t err = dual_imu_update(dual_dev, &result);
        if (err != ICM42688_OK) {
            vTaskDelay(pdMS_TO_TICKS(1));
            continue;
        }

        /* 聚合: 每帧数据存入缓冲 */
        {
            static uint32_t agg_count = 0;
            static net_aggregated_packet_t agg_pkt = {
                .magic = {'I', 'M', 'U', 'A'},
            };
            agg_pkt.node_id = s_node_id;

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

            /* 攒满 5 帧 → 异步队列投递 (非阻塞) */
            if (agg_pkt.frame_count >= NET_AGGREGATE_FRAMES) {
                net_udp_send_aggregated(&agg_pkt);
                agg_pkt.frame_count = 0;
                agg_count++;
                if (agg_count % 200 == 0) {
                    ESP_LOGI(TAG, "TX #%lu QW:%.3f TX_OK:%lu",
                             (unsigned long)agg_count,
                             result.quat.w,
                             (unsigned long)net_espnow_get_send_ok());
                }
            }
        }
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "=== 双 ICM-42688-P + ESP-NOW ===");

    /* ================================================================
     *  LDO 强控上电时序 (任务2)
     *  在任何 SPI/IMU 初始化之前, 先使能 LDO 并等待 50ms,
     *  确保传感器端电压稳压完成、芯片内部复位彻底结束,
     *  杜绝错过首个中断沿的隐患。
     * ================================================================ */
    gpio_config_t ldo_io = {
        .pin_bit_mask = (1ULL << PIN_LDO_EN1) | (1ULL << PIN_LDO_EN2),
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    gpio_config(&ldo_io);
    gpio_set_level(PIN_LDO_EN1, 1);  /* 使能 IMU-A LDO */
    gpio_set_level(PIN_LDO_EN2, 1);  /* 使能 IMU-B LDO */
    ESP_LOGI(TAG, "LDO EN1(IO%d) + EN2(IO%d) 拉高, 等待 50ms 稳压...", PIN_LDO_EN1, PIN_LDO_EN2);
    vTaskDelay(pdMS_TO_TICKS(50));
    ESP_LOGI(TAG, "LDO 稳压完成, 开始 SPI 初始化");

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
    if (s_node_id == 0xFF) s_node_id = 0xFE;  /* 避开广播保留字 0xFF */
    net_set_node_id(s_node_id);
    net_time_sync_init();
    ESP_LOGI(TAG, "ESP-NOW 就绪 (MAC:%02X:%02X:%02X:%02X:%02X:%02X → node_id=0x%02X)",
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5], s_node_id);

    /* ---- 8. 创建高优先级 IMU 采集任务 (Core1, 优先级15) ---- */
    xTaskCreatePinnedToCore(imu_task, "imu_task", 8192, &dual_dev, 15, NULL, 1);
    ESP_LOGI(TAG, "imu_task 已创建 (Core1, 优先级15, 栈8KB)");

    /* app_main 释放 CPU, 不再占用 */
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
