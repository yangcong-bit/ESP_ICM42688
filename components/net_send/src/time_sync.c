/**
 * @file time_sync.c
 * @brief 全局时钟同步实现
 */

#include "time_sync.h"
#include "esp_log.h"
#include "esp_timer.h"
#include <string.h>

static const char *TAG = "time_sync";

/* 同步超时: 超过 5 秒未同步则认为失效 */
#define SYNC_TIMEOUT_US  (5000000LL)

/* ============================================================
 *  节点端 API
 * ============================================================ */

void time_sync_init(time_sync_state_t *state)
{
    memset(state, 0, sizeof(*state));
    state->offset_us = 0;
    state->synchronized = false;
    portMUX_INITIALIZE(&state->offset_mux);
    ESP_LOGI(TAG, "Time sync module initialized (node side)");
}

bool time_sync_handle_rx(time_sync_state_t *state,
                          const uint8_t *pkt, int pkt_len)
{
    if (!state || !pkt || pkt_len < (int)sizeof(timesync_packet_t)) {
        return false;
    }

    const timesync_packet_t *ts_pkt = (const timesync_packet_t *)pkt;

    switch (ts_pkt->type) {
    case TIMESYNC_START: {
        /* 主机发来 SYNC_START, 节点需要回复 */
        ESP_LOGD(TAG, "SYNC_START seq=%lu from host (host_time=%lld us)",
                 (unsigned long)ts_pkt->seq, (long long)ts_pkt->host_time_us);

        /* 不在这里直接回复 — 由上层调用 time_sync_gen_reply() 生成回复包
         * 因为 ESP-NOW 的回复需要在 recv 回调之外发送 (避免死锁) */
        return true;  /* 告知上层: 这是一个需要回复的包 */
    }

    case TIMESYNC_APPLY: {
        /* 主机下发计算好的 offset */
        if (ts_pkt->seq <= state->last_sync_seq) {
            /* 过期的包, 忽略 */
            return false;
        }

        /* 64位原子写入: ESP32-S3 是 32位架构, int64_t 需要临界区保护 */
        portENTER_CRITICAL(&state->offset_mux);
        state->offset_us = ts_pkt->offset_us;
        portEXIT_CRITICAL(&state->offset_mux);
        state->last_sync_seq = ts_pkt->seq;
        state->sync_count++;
        state->synchronized = true;
        state->last_sync_local_us = esp_timer_get_time();

        ESP_LOGI(TAG, "SYNC_APPLY: offset=%lld us (seq=%lu, sync #%lu)",
                 (long long)state->offset_us,
                 (unsigned long)ts_pkt->seq,
                 (unsigned long)state->sync_count);
        return true;
    }

    case TIMESYNC_HEARTBEAT: {
        /* 心跳: 刷新同步有效性 */
        state->last_sync_local_us = esp_timer_get_time();
        return true;
    }

    default:
        return false;  /* 不是同步包 */
    }
}

void time_sync_gen_reply(uint8_t *reply_buf, int *reply_len,
                          const timesync_packet_t *src_pkt, uint8_t node_id)
{
    if (!reply_buf || !src_pkt || !reply_len) return;

    timesync_packet_t *reply = (timesync_packet_t *)reply_buf;
    reply->type = TIMESYNC_REPLY;
    reply->node_id = node_id;
    reply->seq = src_pkt->seq;
    reply->host_time_us = src_pkt->host_time_us;  /* 透传主机时间 */
    reply->node_time_us = esp_timer_get_time();     /* 记录本地收到的时间 */
    reply->offset_us = 0;

    *reply_len = sizeof(timesync_packet_t);
}

int64_t time_sync_get_time(const time_sync_state_t *state)
{
    int64_t local_us = esp_timer_get_time();
    if (state && state->synchronized) {
        return local_us + state->offset_us;
    }
    return local_us;  /* 未同步, 返回本地时间 */
}

bool time_sync_is_valid(const time_sync_state_t *state)
{
    if (!state || !state->synchronized) return false;
    int64_t now = esp_timer_get_time();
    return (now - state->last_sync_local_us) < SYNC_TIMEOUT_US;
}

/* ============================================================
 *  主机端 API
 * ============================================================ */

void time_sync_host_gen_start(time_sync_host_t *host,
                               uint8_t *buf, int *buf_len, uint8_t node_id)
{
    if (!host || !buf || !buf_len) return;

    host->seq++;
    host->last_send_us = esp_timer_get_time();

    timesync_packet_t *pkt = (timesync_packet_t *)buf;
    pkt->type = TIMESYNC_START;
    pkt->node_id = node_id;
    pkt->seq = host->seq;
    pkt->host_time_us = host->last_send_us;
    pkt->node_time_us = 0;
    pkt->offset_us = 0;

    *buf_len = sizeof(timesync_packet_t);
}

bool time_sync_host_handle_reply(time_sync_host_t *host,
                                  const timesync_packet_t *reply,
                                  uint8_t *apply_buf, int *apply_len)
{
    if (!host || !reply || !apply_buf || !apply_len) return false;
    if (reply->type != TIMESYNC_REPLY) return false;

    int64_t now = esp_timer_get_time();
    /* RTT = 本地当前时间 - 之前发送 START 的时间 */
    host->rtt_us = now - host->last_send_us;

    /*
     * 计算 offset:
     *   T_host_send = host->last_send_us
     *   T_node_rx   = reply->node_time_us
     *   T_host_rx   = now
     *   RTT = T_host_rx - T_host_send
     *   offset = T_host_send + RTT/2 - T_node_rx
     *          = (T_host_send + T_host_rx) / 2 - T_node_rx
     */
    int64_t offset = (host->last_send_us + now) / 2 - reply->node_time_us;

    /* 生成 SYNC_APPLY 包 */
    timesync_packet_t *apply = (timesync_packet_t *)apply_buf;
    apply->type = TIMESYNC_APPLY;
    apply->node_id = reply->node_id;
    apply->seq = host->seq;
    apply->host_time_us = now;
    apply->node_time_us = reply->node_time_us;
    apply->offset_us = offset;

    *apply_len = sizeof(timesync_packet_t);

    ESP_LOGI(TAG, "SYNC: node=%d, RTT=%lld us, offset=%lld us",
             reply->node_id, (long long)host->rtt_us, (long long)offset);

    return true;
}

void time_sync_host_gen_heartbeat(time_sync_host_t *host,
                                   uint8_t *buf, int *buf_len)
{
    if (!host || !buf || !buf_len) return;

    timesync_packet_t *pkt = (timesync_packet_t *)buf;
    pkt->type = TIMESYNC_HEARTBEAT;
    pkt->node_id = 0xFF;  /* 广播 */
    pkt->seq = host->seq;
    pkt->host_time_us = esp_timer_get_time();
    pkt->node_time_us = 0;
    pkt->offset_us = 0;

    *buf_len = sizeof(timesync_packet_t);
}
