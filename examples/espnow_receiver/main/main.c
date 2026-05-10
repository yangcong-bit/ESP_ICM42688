/**
 * @file main.c
 * @brief ESP-NOW 主控节点 (Master) — 时钟同步 + TDMA 调度
 *
 * 功能:
 *   1. 周期广播 SYNC_START (1Hz) 维持全局时钟
 *   2. 接收 SYNC_REPLY → 计算 offset → 回复 SYNC_APPLY
 *   3. 启动时广播 TIMESYNC_TDMA_CFG 配置所有节点的时隙
 *   4. 接收并解析 IMU 聚合数据包 (248B, 5帧)
 */

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_now.h"
#include "esp_event.h"
#include "esp_timer.h"

static const char *TAG = "master";

/* ============================================================
 *  协议定义 (与节点端 time_sync.h 保持一致)
 * ============================================================ */
#define TIMESYNC_START      0x01
#define TIMESYNC_REPLY      0x02
#define TIMESYNC_APPLY      0x03
#define TIMESYNC_HEARTBEAT  0x04
#define TIMESYNC_TDMA_CFG   0x05

typedef struct __attribute__((packed)) {
    uint8_t  type;
    uint8_t  node_id;
    uint32_t seq;
    int64_t  host_time_us;
    int64_t  node_time_us;
    int64_t  offset_us;
} timesync_packet_t;

typedef struct __attribute__((packed)) {
    uint8_t  type;           /* TIMESYNC_TDMA_CFG */
    uint8_t  node_id;        /* 0xFF = 广播 */
    uint32_t period_us;      /* 总周期 */
    uint32_t offset_us;      /* 时隙偏移 */
    uint32_t window_us;      /* 窗口长度 */
    uint8_t  enable;         /* 1=开启, 0=关闭 */
} timesync_tdma_cfg_pkt_t;

/* IMU 聚合帧结构 (与节点端一致) */
#define NET_AGGREGATE_FRAMES  5
typedef struct __attribute__((packed)) {
    float    accel[3];
    float    gyro[3];
    float    quat[4];
    float    euler[3];
    float    temp;
    uint64_t timestamp_us;
} net_frame_t;

typedef struct __attribute__((packed)) {
    uint8_t  magic[4];
    uint8_t  node_id;
    uint8_t  frame_count;
    uint16_t reserved;
    net_frame_t frames[NET_AGGREGATE_FRAMES];
} net_aggregated_packet_t;

/* 广播 MAC */
static const uint8_t s_broadcast_mac[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

/* 主控状态 */
typedef struct {
    uint32_t seq;
    int64_t  last_sync_us;
    uint32_t node_count;        /* 已发现的节点数 */
} host_state_t;

static host_state_t s_host = {0};

/* 统计 */
static uint32_t s_imu_count = 0;
static uint32_t s_reply_count = 0;

/* ============================================================
 *  接收回调: 分类处理 IMU 数据 / SYNC_REPLY
 * ============================================================ */
static void espnow_recv_cb(const esp_now_recv_info_t *info,
                            const uint8_t *data, int len)
{
    if (!data || !info) return;

    /* 1. 处理时钟同步回复 */
    if (len == sizeof(timesync_packet_t)) {
        const timesync_packet_t *pkt = (const timesync_packet_t *)data;
        if (pkt->type == TIMESYNC_REPLY) {
            int64_t now_us = esp_timer_get_time();

            /* 计算 offset: (T_host_send + T_host_rx) / 2 - T_node_rx */
            int64_t offset = (s_host.last_sync_us + now_us) / 2 - pkt->node_time_us;

            /* 构造 SYNC_APPLY */
            timesync_packet_t apply = {
                .type = TIMESYNC_APPLY,
                .node_id = pkt->node_id,
                .seq = s_host.seq,
                .host_time_us = now_us,
                .node_time_us = pkt->node_time_us,
                .offset_us = offset
            };

            esp_now_send(info->src_addr, (const uint8_t *)&apply, sizeof(apply));
            s_reply_count++;

            if (s_reply_count % 10 == 1) {
                ESP_LOGI(TAG, "SYNC_APPLY → node 0x%02X offset=%lld us (RTT≈%lld us)",
                         pkt->node_id, (long long)offset,
                         (long long)(now_us - s_host.last_sync_us));
            }
            return;
        }
    }

    /* 2. 处理 IMU 聚合数据包 */
    if (len >= 8 && data[0] == 'I' && data[1] == 'M' && data[2] == 'U' && data[3] == 'A') {
        const net_aggregated_packet_t *agg = (const net_aggregated_packet_t *)data;
        s_imu_count++;

        if (s_imu_count % 50 == 0) {
            ESP_LOGI(TAG, "[NODE 0x%02X] %d帧 | #1: QW=%.3f A=[%.2f,%.2f,%.2f] | ts=%llu",
                     agg->node_id, agg->frame_count,
                     agg->frames[0].quat[0],
                     agg->frames[0].accel[0], agg->frames[0].accel[1], agg->frames[0].accel[2],
                     (unsigned long long)agg->frames[0].timestamp_us);
        }

        /* 首包打印来源 MAC */
        if (s_imu_count == 1) {
            ESP_LOGI(TAG, "首包来自: %02X:%02X:%02X:%02X:%02X:%02X node_id=0x%02X",
                     info->src_addr[0], info->src_addr[1], info->src_addr[2],
                     info->src_addr[3], info->src_addr[4], info->src_addr[5],
                     agg->node_id);
        }
    }
}

/* ============================================================
 *  主控调度任务: 广播 TDMA 配置 + 周期 SYNC_START
 * ============================================================ */
static void master_ctrl_task(void *arg)
{
    ESP_LOGI(TAG, "Master Ctrl: 广播 TDMA 配置 + 时钟同步...");

    /* 1. 启动时连续广播 TDMA 配置 (确保节点收到) */
    timesync_tdma_cfg_pkt_t tdma_cfg = {
        .type      = TIMESYNC_TDMA_CFG,
        .node_id   = 0xFF,       /* 广播给所有节点 */
        .period_us = 5000,       /* 200Hz 周期 */
        .offset_us = 0,          /* 节点端自动按 node_id 计算 */
        .window_us = 350,        /* 发送窗口 */
        .enable    = 1           /* 开启 TDMA */
    };

    for (int i = 0; i < 5; i++) {
        esp_now_send(s_broadcast_mac, (const uint8_t *)&tdma_cfg, sizeof(tdma_cfg));
        ESP_LOGI(TAG, "TDMA CFG 广播 #%d (200Hz, 350us窗口)", i + 1);
        vTaskDelay(pdMS_TO_TICKS(200));
    }

    ESP_LOGI(TAG, "TDMA 配置下发完成, 开始周期时钟同步 (1Hz)...");

    /* 2. 周期性广播 SYNC_START (1Hz) */
    while (1) {
        s_host.seq++;
        s_host.last_sync_us = esp_timer_get_time();

        timesync_packet_t sync_start = {
            .type = TIMESYNC_START,
            .node_id = 0xFF,        /* 广播 */
            .seq = s_host.seq,
            .host_time_us = s_host.last_sync_us,
            .node_time_us = 0,
            .offset_us = 0
        };

        esp_now_send(s_broadcast_mac, (const uint8_t *)&sync_start, sizeof(sync_start));

        if (s_host.seq % 10 == 0) {
            ESP_LOGI(TAG, "SYNC_START seq=%lu | IMU:%lu REPLY:%lu",
                     (unsigned long)s_host.seq,
                     (unsigned long)s_imu_count,
                     (unsigned long)s_reply_count);
        }

        vTaskDelay(pdMS_TO_TICKS(1000));
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
    ESP_LOGI(TAG, "Master MAC: %02X:%02X:%02X:%02X:%02X:%02X",
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
}

/* ============================================================
 *  app_main
 * ============================================================ */
void app_main(void)
{
    ESP_LOGI(TAG, "=== ESP-NOW Master (TDMA + 时钟同步) ===");

    /* 1. WiFi 初始化 */
    wifi_init();

    /* 2. ESP-NOW 初始化 */
    ESP_ERROR_CHECK(esp_now_init());
    ESP_ERROR_CHECK(esp_now_register_recv_cb(espnow_recv_cb));

    /* 添加广播 Peer */
    esp_now_peer_info_t peer_info = {0};
    memcpy(peer_info.peer_addr, s_broadcast_mac, 6);
    peer_info.channel = 0;
    peer_info.encrypt = false;
    ESP_ERROR_CHECK(esp_now_add_peer(&peer_info));

    /* 3. 启动主控调度任务 */
    xTaskCreate(master_ctrl_task, "master_ctrl", 4096, NULL, 5, NULL);
}

    ESP_LOGI(TAG, "等待接收 IMU 数据包 (sizeof=%d bytes)...", (int)sizeof(imu_packet_t));
    ESP_LOGI(TAG, "将发送端烧录为主程序, 此端保持监听");

    /* 主循环空闲 (回调处理所有逻辑) */
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));

        /* 每秒打印统计 */
        ESP_LOGI(TAG, "已接收 %lu 包", (unsigned long)s_pkt_count);
    }
}
