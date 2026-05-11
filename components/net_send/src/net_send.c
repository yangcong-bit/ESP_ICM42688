/**
 * @file net_send.c
 * @brief ESP-NOW 广播数据发送实现 (无需路由器)
 *
 * 使用 ESP-NOW 底层广播协议, 将 IMU 数据直接发送到
 * 同信道的所有 ESP32 接收设备, 无需 WiFi 路由器。
 *
 * 接收端只需监听 ESP-NOW 数据帧即可。
 */

#include "net_send.h"
#include "time_sync.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_now.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "esp_event.h"
#include "esp_rom_sys.h"
#include "esp_timer.h"
#include <string.h>
#include <stdio.h>

static const char *TAG = "net_send";

/* ============================================================
 *  ESP-NOW 状态
 * ============================================================ */
static bool s_espnow_init_done = false;
static uint32_t s_send_ok_count = 0;
static uint32_t s_send_fail_count = 0;
static time_sync_state_t s_time_sync = {0};
static uint8_t s_node_id = 0;

/* 广播目标 MAC 地址 */
static const uint8_t s_broadcast_mac[ESP_NOW_ETH_ALEN] = {
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF
};

/* ============================================================
 *  异步发送队列 (消除回调中 esp_now_send + 主循环阻塞)
 * ============================================================ */
#define SYNC_REPLY_QUEUE_SIZE   4
#define IMU_SEND_QUEUE_SIZE     40  /* 200ms 抗干扰缓冲池 (200Hz × 0.2s) */

typedef struct {
    uint8_t  dest_mac[ESP_NOW_ETH_ALEN];
    uint8_t  payload[sizeof(timesync_packet_t)];
    int      payload_len;
} sync_reply_item_t;

typedef struct {
    uint8_t  data[sizeof(net_aggregated_packet_t)];
    int      data_len;
} imu_send_item_t;

static QueueHandle_t s_sync_reply_queue = NULL;
static QueueHandle_t s_imu_send_queue = NULL;
static TaskHandle_t  s_espnow_task_handle = NULL;

/* ESP-NOW 专用发送任务 (任务级上下文, 非回调, 非主循环) */
static void espnow_send_task(void *arg)
{
    sync_reply_item_t reply;
    imu_send_item_t   imu_pkt;

    while (1) {
        /* 优先处理时间同步回复 (延迟敏感) */
        if (xQueueReceive(s_sync_reply_queue, &reply, 0) == pdTRUE) {
            esp_now_send(reply.dest_mac, reply.payload, reply.payload_len);
            continue;
        }
        /* 其次处理 IMU 聚合数据 (TDMA 时隙控制) */
        if (xQueueReceive(s_imu_send_queue, &imu_pkt, pdMS_TO_TICKS(10)) == pdTRUE) {
            /* TDMA: 混合调度 (Hybrid Sleep-Spin)
             * > 2ms: vTaskDelay 挂起, 释放 Core 0 让 PM 切 80MHz Idle
             * < 2ms: esp_rom_delay_us 微秒级自旋, 保证时隙精度不撕裂 */
            while (1) {
                int64_t wait_us = time_sync_us_until_next_slot(&s_time_sync);

                /* 已在时隙内或未同步 → 立即发送 */
                if (wait_us <= 0) break;

                if (wait_us > 2000) {
                    /* 粗调: 距离时隙较远, 让出 CPU 给 IDLE 任务 */
                    uint32_t delay_ticks = pdMS_TO_TICKS((wait_us - 1000) / 1000);
                    if (delay_ticks > 0) {
                        vTaskDelay(delay_ticks);
                    } else {
                        taskYIELD();
                    }
                } else {
                    /* 微调: 距离时隙 <2ms, 高精度自旋防抖 */
                    esp_rom_delay_us(50);
                }
            }
            esp_now_send(s_broadcast_mac, imu_pkt.data, imu_pkt.data_len);
        }
    }
}

/* ============================================================
 *  ESP-NOW 发送回调
 * ============================================================ */
static void espnow_send_cb(const esp_now_send_info_t *info, esp_now_send_status_t status)
{
    if (status == ESP_NOW_SEND_SUCCESS) {
        s_send_ok_count++;
    } else {
        s_send_fail_count++;
        if (s_send_fail_count % 50 == 1) {
            ESP_LOGW(TAG, "ESP-NOW send failed (total: %lu)", (unsigned long)s_send_fail_count);
        }
    }
}

/* ============================================================
 *  ESP-NOW 接收回调 (禁止在此调用 esp_now_send!)
 * ============================================================ */
static void espnow_recv_cb(const esp_now_recv_info_t *info, const uint8_t *data, int len)
{
    if (!info || !data || len <= 0) return;

    /* 检查是否是时间同步包 */
    if (len >= (int)sizeof(timesync_packet_t)) {
        const timesync_packet_t *ts_pkt = (const timesync_packet_t *)data;
        if (ts_pkt->type >= TIMESYNC_START && ts_pkt->type <= TIMESYNC_HEARTBEAT) {
            bool handled = time_sync_handle_rx(&s_time_sync, data, len);
            if (handled && ts_pkt->type == TIMESYNC_START && s_sync_reply_queue) {
                /* 严禁在回调中调用 esp_now_send, 压入队列由任务级发送 */
                sync_reply_item_t item;
                memcpy(item.dest_mac, info->src_addr, ESP_NOW_ETH_ALEN);
                time_sync_gen_reply(item.payload, &item.payload_len, ts_pkt, s_node_id);
                xQueueSend(s_sync_reply_queue, &item, 0);  /* 非阻塞, 满则丢弃 */
            }
            return;
        }
    }
    /* 非同步包: 此处可扩展 */
}

/* ============================================================
 *  WiFi 初始化 (ESP-NOW 需要底层 WiFi, 但不需要连接路由器)
 * ============================================================ */
static bool wifi_lowlevel_init(void)
{
    /* 1. 初始化 NVS (WiFi 驱动需要 NVS 存储射频校准数据) */
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    /* 2. 初始化底层网络接口和默认事件循环 */
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    /* 3. 初始化 WiFi 驱动 (STA 模式, 但不连接任何 AP) */
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ret = esp_wifi_init(&cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "WiFi init failed: %s", esp_err_to_name(ret));
        return false;
    }

    ret = esp_wifi_set_mode(WIFI_MODE_STA);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "WiFi set mode failed: %s", esp_err_to_name(ret));
        return false;
    }

    ret = esp_wifi_start();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "WiFi start failed: %s", esp_err_to_name(ret));
        return false;
    }

    /* 获取本机 MAC 地址 */
    uint8_t mac[ESP_NOW_ETH_ALEN];
    esp_wifi_get_mac(WIFI_IF_STA, mac);
    ESP_LOGI(TAG, "WiFi MAC: %02X:%02X:%02X:%02X:%02X:%02X",
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

    /* 设置发射功率 (可选, 最大 20dBm) */
    esp_wifi_set_max_tx_power(78);  /* 78 = 19.5dBm */

    return true;
}

/* ============================================================
 *  API — WiFi 初始化 (兼容旧接口, 内部调用 WiFi 低层初始化)
 * ============================================================ */
bool net_wifi_init(const net_wifi_cfg_t *cfg)
{
    /* ESP-NOW 模式下忽略 SSID/密码, 仅初始化底层 WiFi */
    ESP_LOGI(TAG, "ESP-NOW 模式: 初始化底层 WiFi (无需路由器)");
    return wifi_lowlevel_init();
}

bool net_wifi_is_connected(void)
{
    /* ESP-NOW 无需连接路由器, 始终返回 true */
    return true;
}

const char *net_wifi_get_ip(void)
{
    /* ESP-NOW 无 IP 地址 */
    return "N/A (ESP-NOW)";
}

/* ============================================================
 *  API — ESP-NOW 初始化
 * ============================================================ */
bool net_udp_init(const net_udp_cfg_t *cfg)
{
    /* 兼容旧接口, 忽略 UDP 参数, 初始化 ESP-NOW */
    if (s_espnow_init_done) return true;

    /* 初始化异步发送队列 + 发送任务 */
    if (!s_sync_reply_queue) {
        s_sync_reply_queue = xQueueCreate(SYNC_REPLY_QUEUE_SIZE, sizeof(sync_reply_item_t));
        s_imu_send_queue = xQueueCreate(IMU_SEND_QUEUE_SIZE, sizeof(imu_send_item_t));
        xTaskCreate(espnow_send_task, "espnow_tx", 4096, NULL, 5, &s_espnow_task_handle);
    }

    /* 初始化 ESP-NOW */
    esp_err_t ret = esp_now_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "ESP-NOW init failed: %s", esp_err_to_name(ret));
        return false;
    }

    /* 注册发送/接收回调 */
    esp_now_register_send_cb(espnow_send_cb);
    esp_now_register_recv_cb(espnow_recv_cb);

    /* 添加广播 peer */
    esp_now_peer_info_t peer_info = {0};
    memcpy(peer_info.peer_addr, s_broadcast_mac, ESP_NOW_ETH_ALEN);
    peer_info.channel = 0;  /* 使用当前 WiFi 信道 */
    peer_info.encrypt = false;

    ret = esp_now_add_peer(&peer_info);
    if (ret != ESP_OK && ret != ESP_ERR_ESPNOW_EXIST) {
        ESP_LOGE(TAG, "ESP-NOW add broadcast peer failed: %s", esp_err_to_name(ret));
        return false;
    }

    s_espnow_init_done = true;

    if (cfg) {
        ESP_LOGI(TAG, "ESP-NOW broadcast initialized (target ignored in ESP-NOW mode)");
    } else {
        ESP_LOGI(TAG, "ESP-NOW broadcast initialized");
    }

    return true;
}

/* ============================================================
 *  API — ESP-NOW 发送 IMU 数据包
 * ============================================================ */
bool net_udp_send_imu(const net_imu_packet_t *pkt)
{
    if (!s_espnow_init_done || !pkt) return false;

    esp_err_t ret = esp_now_send(s_broadcast_mac,
                                  (const uint8_t *)pkt,
                                  sizeof(net_imu_packet_t));
    return (ret == ESP_OK);
}

bool net_udp_send_raw(const char *data, size_t len)
{
    if (!s_espnow_init_done || !data || len == 0) return false;

    esp_err_t ret = esp_now_send(s_broadcast_mac,
                                  (const uint8_t *)data, len);
    return (ret == ESP_OK);
}

/* ============================================================
 *  API — HTTP (不适用 ESP-NOW, 保留接口兼容)
 * ============================================================ */
bool net_http_init(const net_http_cfg_t *cfg)
{
    ESP_LOGW(TAG, "HTTP not available in ESP-NOW mode, using broadcast instead");
    /* 降级为 ESP-NOW 广播 */
    return net_udp_init(NULL);
}

bool net_http_send_imu(const net_imu_packet_t *pkt)
{
    /* 降级为 ESP-NOW 广播 */
    return net_udp_send_imu(pkt);
}

bool net_http_send_json(const char *json_str)
{
    if (!s_espnow_init_done || !json_str) return false;
    return net_udp_send_raw(json_str, strlen(json_str));
}

/* ============================================================
 *  API — 关闭
 * ============================================================ */
void net_deinit(void)
{
    if (s_espnow_init_done) {
        esp_now_deinit();
        s_espnow_init_done = false;
    }
    if (s_espnow_task_handle) {
        vTaskDelete(s_espnow_task_handle);
        s_espnow_task_handle = NULL;
    }
    if (s_sync_reply_queue) {
        vQueueDelete(s_sync_reply_queue);
        s_sync_reply_queue = NULL;
    }
    if (s_imu_send_queue) {
        vQueueDelete(s_imu_send_queue);
        s_imu_send_queue = NULL;
    }
    esp_wifi_stop();
    ESP_LOGI(TAG, "ESP-NOW deinitialized");
}

/* ============================================================
 *  API — 获取统计信息
 * ============================================================ */
uint32_t net_espnow_get_send_ok(void)
{
    return s_send_ok_count;
}

uint32_t net_espnow_get_send_fail(void)
{
    return s_send_fail_count;
}

/* ============================================================
 *  API — 聚合发送
 * ============================================================ */

bool net_udp_send_aggregated(const net_aggregated_packet_t *agg)
{
    if (!s_espnow_init_done || !agg || agg->frame_count == 0) return false;
    size_t pkt_size = 8 + agg->frame_count * sizeof(net_frame_t);

    /* 异步队列投递: 非阻塞, 满则丢弃 (由 espnow_send_task 负责实际发送) */
    imu_send_item_t item;
    memcpy(item.data, agg, pkt_size);
    item.data_len = pkt_size;
    BaseType_t ret = xQueueSend(s_imu_send_queue, &item, 0);
    return (ret == pdTRUE);
}

/* ============================================================
 *  API — 时间同步
 * ============================================================ */

void net_time_sync_init(void)
{
    time_sync_init(&s_time_sync);
}

void net_set_node_id(uint8_t id)
{
    s_node_id = id;

    /* TDMA 自动配置: 200Hz → period=5000us, 12节点 → 400us/节点, 保护间隔50us */
    uint32_t period_us  = 5000;                         /* 发送周期 200Hz */
    uint32_t slot_us    = period_us / 12;               /* 每节点时隙 400us */
    uint32_t guard_us   = 50;                           /* 保护间隔 50us */
    uint32_t window_us  = slot_us - guard_us;           /* 有效窗口 350us */
    uint32_t offset_us  = (id % 12) * slot_us;          /* 本节点偏移 */
    time_sync_set_tdma(&s_time_sync, true, period_us, offset_us, window_us);
}

int64_t net_get_synced_time(void)
{
    return time_sync_get_time(&s_time_sync);
}

bool net_time_sync_valid(void)
{
    return time_sync_is_valid(&s_time_sync);
}

uint32_t net_time_sync_count(void)
{
    return s_time_sync.sync_count;
}
