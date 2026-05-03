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
#include "esp_event.h"
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
static uint8_t s_node_id = 0;  /* 本节点 ID, 可通过 net_set_node_id 设置 */

/* 广播目标 MAC 地址 (FF:FF:FF:FF:FF:FF = 所有设备) */
static const uint8_t s_broadcast_mac[ESP_NOW_ETH_ALEN] = {
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF
};

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
 *  ESP-NOW 接收回调 (接收端数据, 本端主要发送, 此处仅打印)
 * ============================================================ */
static void espnow_recv_cb(const esp_now_recv_info_t *info, const uint8_t *data, int len)
{
    if (!info || !data || len <= 0) return;

    /* 检查是否是时间同步包 */
    if (len >= (int)sizeof(timesync_packet_t)) {
        const timesync_packet_t *ts_pkt = (const timesync_packet_t *)data;
        if (ts_pkt->type >= TIMESYNC_START && ts_pkt->type <= TIMESYNC_HEARTBEAT) {
            bool handled = time_sync_handle_rx(&s_time_sync, data, len);
            if (handled && ts_pkt->type == TIMESYNC_START) {
                /* 需要回复 SYNC_REPLY — 在回调中用 esp_now_send 发送 */
                uint8_t reply_buf[sizeof(timesync_packet_t)];
                int reply_len = 0;
                time_sync_gen_reply(reply_buf, &reply_len, ts_pkt, s_node_id);
                esp_now_send(info->src_addr, reply_buf, reply_len);
            }
            return;  /* 同步包处理完毕, 不传给上层 */
        }
    }

    /* 非同步包: 此处可扩展处理其他自定义数据 */
}

/* ============================================================
 *  WiFi 初始化 (ESP-NOW 需要底层 WiFi, 但不需要连接路由器)
 * ============================================================ */
static bool wifi_lowlevel_init(void)
{
    /* 初始化 WiFi 驱动 (STA 模式, 但不连接任何 AP) */
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_err_t ret = esp_wifi_init(&cfg);
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
 *  API — 时间同步
 * ============================================================ */

void net_time_sync_init(void)
{
    time_sync_init(&s_time_sync);
}

void net_set_node_id(uint8_t id)
{
    s_node_id = id;
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
