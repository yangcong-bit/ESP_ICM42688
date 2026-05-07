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

#if !CONFIG_IMU_MOCK_MODE
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

    err = dual_imu_calibrate(&dual_dev, 500);
    if (err != ICM42688_OK) {
        ESP_LOGW(TAG, "Cross calibration failed, continuing");
    }
#else
    /* ---- MOCK MODE: 手动构造 dual_dev, 不触碰任何 SPI/GPIO ---- */
    dual_imu_dev_t dual_dev;
    memset(&dual_dev, 0, sizeof(dual_dev));
    dual_dev.imu[0].dev = &imu_a;
    dual_dev.imu[0].online = true;
    dual_dev.imu[1].dev = &imu_b;
    dual_dev.imu[1].online = true;
    dual_dev.initialized = true;
    dual_dev.fused_count = 0;

    /* ESKF 初始化: 单位四元数 + 对角协方差 */
    eskf_init(&dual_dev.eskf_fused, &dual_dev.eskf_state_fused);
    dual_dev.eskf_state_fused.q[0] = 1.0f;
    memset(dual_dev.eskf_fused.P, 0, sizeof(dual_dev.eskf_fused.P));
    for (int i = 0; i < 6; i++) {
        dual_dev.eskf_fused.P[i][i] = 0.001f;
    }
    dual_dev.last_update_us = esp_timer_get_time();
    ESP_LOGI(TAG, "Mock: dual_dev + ESKF 手动初始化完成 Q=[1,0,0,0]");
#endif

    /* ---- 7. 初始化 ESP-NOW 广播 + 时间同步 ---- */
#if !CONFIG_IMU_MOCK_MODE
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
    s_node_id = mac[5];
    net_set_node_id(s_node_id);
    net_time_sync_init();
    ESP_LOGI(TAG, "ESP-NOW 就绪 (MAC:%02X:%02X:%02X:%02X:%02X:%02X → node_id=0x%02X)",
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5], s_node_id);
#else
    /* MOCK MODE: 跳过 WiFi/NVS, 仅本地验证算法链路 */
    s_node_id = 0xFF;
    ESP_LOGW(TAG, "Mock: 跳过 WiFi/NVS 初始化, 仅本地验证 ESKF 链路");
#endif

    /* ---- 8. 主循环 ---- */
    dual_imu_result_t result;
    memset(&result, 0, sizeof(result));

    ESP_LOGW("TRACER", "app_main: 所有初始化完成, 即将进入主循环");

    ESP_LOGI(TAG, "=== 极速四元数模式 ===");

#if CONFIG_IMU_MOCK_MODE
    ESP_LOGW(TAG, "*** MOCK MODE: 纯软件验证 ESKF 链路 (零驱动调用) ***");
#endif

    ESP_LOGW("TRACER", "准备进入 while(1) 主循环...");

    while (1) {
#if CONFIG_IMU_MOCK_MODE
        ESP_LOGW("TRACER", "=== 循环起点 ===");

        /* 1. 模拟数据生成 (纯数学, 无任何驱动调用) */
        float t = esp_timer_get_time() / 1000000.0f;
        float gyro_rads[3] = { 0.1f, 0.0f, 0.5f };
        float accel_g[3] = { sinf(t), cosf(t), 1.0f };
        ESP_LOGW("TRACER", "Mock 数据生成完毕 t=%.3f", t);

        /* 2. 预测步骤 */
        ESP_LOGW("TRACER", "准备调用 eskf_predict...");
        eskf_predict(&dual_dev.eskf_fused, &dual_dev.eskf_state_fused, gyro_rads, 0.01f);
        ESP_LOGW("TRACER", "eskf_predict 执行完毕");

        /* 3. 更新步骤 */
        ESP_LOGW("TRACER", "准备调用 eskf_update_accel...");
        eskf_update_accel(&dual_dev.eskf_fused, &dual_dev.eskf_state_fused, accel_g);
        ESP_LOGW("TRACER", "eskf_update_accel 执行完毕");

        /* 4. 读取结果 */
        ESP_LOGW("TRACER", "四元数: W=%.3f X=%.3f Y=%.3f Z=%.3f",
                 dual_dev.eskf_state_fused.q[0],
                 dual_dev.eskf_state_fused.q[1],
                 dual_dev.eskf_state_fused.q[2],
                 dual_dev.eskf_state_fused.q[3]);

        /* 5. 收尾延迟 (10ms, 让出 CPU 给 IDLE) */
        ESP_LOGW("TRACER", "准备 vTaskDelay(10ms)...");
        vTaskDelay(pdMS_TO_TICKS(10));
        ESP_LOGW("TRACER", "=== 循环终点 (Delay 结束) ===");

#else
        /* ---- 真实硬件模式 ---- */
        icm42688_err_t drdy_a = icm42688_wait_drdy(&imu_a, 2);
        icm42688_err_t drdy_b = icm42688_wait_drdy(&imu_b, 2);
        if (drdy_a == ICM42688_ERR_TIMEOUT && drdy_b == ICM42688_ERR_TIMEOUT) {
            vTaskDelay(pdMS_TO_TICKS(1));
            continue;
        }
        err = dual_imu_update(&dual_dev, &result);
        if (err != ICM42688_OK) {
            vTaskDelay(pdMS_TO_TICKS(1));
            continue;
        }
#endif
    }
}
