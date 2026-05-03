/**
 * @file espnow_receiver.c
 * @brief ESP-NOW 数据接收端示例
 *
 * 将此代码烧录到另一块 ESP32, 它会监听并打印
 * 来自发送端的 IMU 数据包。
 *
 * 硬件: 任意 ESP32/ESP32-S3
 * 接线: 无需接线, 无线接收
 * 烧录: idf.py flash monitor
 */

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_now.h"
#include "esp_event.h"

static const char *TAG = "espnow_rx";

/* 与发送端一致的数据包结构 */
typedef struct {
    float    accel[3];
    float    gyro[3];
    float    temp;
    float    quat[4];
    float    euler[3];
    uint64_t timestamp_us;
} imu_packet_t;

/* 统计 */
static uint32_t s_pkt_count = 0;
static int64_t  s_last_time = 0;

/* ============================================================
 *  接收回调
 * ============================================================ */
static void espnow_recv_cb(const esp_now_recv_info_t *info,
                            const uint8_t *data, int len)
{
    if (!data || len != sizeof(imu_packet_t)) return;

    imu_packet_t pkt;
    memcpy(&pkt, data, sizeof(pkt));

    s_pkt_count++;
    int64_t now = esp_timer_get_time();

    /* 每 50 包打印一次, 或间隔 > 500ms */
    if (s_pkt_count % 50 == 0 || (now - s_last_time) > 500000) {
        s_last_time = now;

        ESP_LOGI(TAG, "[%lu] R=%6.1f° P=%6.1f° Y=%6.1f° | "
                 "A=[%.3f, %.3f, %.3f] g | "
                 "G=[%.1f, %.1f, %.1f] dps | "
                 "T=%.1f°C | ts=%llu",
                 (unsigned long)s_pkt_count,
                 pkt.euler[0], pkt.euler[1], pkt.euler[2],
                 pkt.accel[0], pkt.accel[1], pkt.accel[2],
                 pkt.gyro[0], pkt.gyro[1], pkt.gyro[2],
                 pkt.temp, (unsigned long long)pkt.timestamp_us);
    }

    /* MAC 来源信息 (仅首包) */
    if (s_pkt_count == 1 && info) {
        ESP_LOGI(TAG, "收到首包, 来自: %02X:%02X:%02X:%02X:%02X:%02X",
                 info->src_addr[0], info->src_addr[1], info->src_addr[2],
                 info->src_addr[3], info->src_addr[4], info->src_addr[5]);
    }
}

/* ============================================================
 *  WiFi 低层初始化
 * ============================================================ */
static void wifi_init(void)
{
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());

    uint8_t mac[6];
    esp_wifi_get_mac(WIFI_IF_STA, mac);
    ESP_LOGI(TAG, "接收端 MAC: %02X:%02X:%02X:%02X:%02X:%02X",
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
}

/* ============================================================
 *  app_main
 * ============================================================ */
void app_main(void)
{
    ESP_LOGI(TAG, "=== ESP-NOW IMU 接收端 ===");

    /* 初始化 WiFi */
    wifi_init();

    /* 初始化 ESP-NOW */
    ESP_ERROR_CHECK(esp_now_init());
    ESP_ERROR_CHECK(esp_now_register_recv_cb(espnow_recv_cb));

    ESP_LOGI(TAG, "等待接收 IMU 数据包 (sizeof=%d bytes)...", (int)sizeof(imu_packet_t));
    ESP_LOGI(TAG, "将发送端烧录为主程序, 此端保持监听");

    /* 主循环空闲 (回调处理所有逻辑) */
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));

        /* 每秒打印统计 */
        ESP_LOGI(TAG, "已接收 %lu 包", (unsigned long)s_pkt_count);
    }
}
