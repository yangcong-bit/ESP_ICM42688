/**
 * @file time_sync.h
 * @brief 全局时钟同步模块
 *
 * 实现原理:
 *   1. 主机 (RK3566/PC) 每秒广播一个 SYNC_START 包 (携带主机时间戳 T_host)
 *   2. 节点收到后记录本地时间 T_local_rx, 回复 SYNC_REPLY (携带 T_local_rx)
 *   3. 主机收到后计算 RTT, 广播 SYNC_APPLY (携带 offset = T_host - T_local_rx + RTT/2)
 *   4. 节点应用 offset, 后续数据包时间戳 = 本地时间 + offset
 *
 * 同步精度: ~10-50μs (ESP-NOW 延迟 + 中断抖动)
 */

#ifndef TIME_SYNC_H
#define TIME_SYNC_H

#include <stdint.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "portmacro.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================
 *  同步包类型 (ESP-NOW 自定义帧)
 * ============================================================ */

/* 自定义 ESP-NOW 保留类型标识 (0~250) */
#define TIMESYNC_ESPNOW_TYPE  0xFD  /* 保留给时间同步 */

/**
 * 同步包结构 (通过 ESP-NOW 自定义 action frame 发送)
 *
 * 主机 → 节点:  SYNC_START
 * 节点 → 主机:  SYNC_REPLY
 * 主机 → 节点:  SYNC_APPLY
 */
typedef struct __attribute__((packed)) {
    uint8_t  type;           /* 包类型 */
    uint8_t  node_id;        /* 节点 ID (0xFF = 广播) */
    uint32_t seq;            /* 序列号 */
    int64_t  host_time_us;   /* 主机时间戳 (μs) */
    int64_t  node_time_us;   /* 节点时间戳 (μs), 仅 SYNC_REPLY 使用 */
    int64_t  offset_us;      /* 时间偏移 (μs), 仅 SYNC_APPLY 使用 */
} timesync_packet_t;

/* 包类型定义 */
#define TIMESYNC_START    0x01  /* 主机 → 广播: 开始同步 */
#define TIMESYNC_REPLY    0x02  /* 节点 → 主机: 回复 */
#define TIMESYNC_APPLY    0x03  /* 主机 → 广播: 下发 offset */
#define TIMESYNC_HEARTBEAT 0x04 /* 主机 → 广播: 心跳 (维持同步) */
#define TIMESYNC_TDMA_CFG  0x05 /* 主机 → 节点: 下发 TDMA 时隙配置 */

/* ============================================================
 *  节点端 API (传感器节点)
 * ============================================================ */
typedef struct {
    volatile int64_t offset_us;  /* 当前时间偏移 (host - local), 需临界区保护 */
    portMUX_TYPE    offset_mux;  /* 64位读写自旋锁 */
    uint32_t last_sync_seq;      /* 最后同步的序列号 */
    uint32_t sync_count;         /* 同步次数 */
    bool     synchronized;       /* 是否已同步 */
    uint64_t last_sync_local_us; /* 上次同步时的本地时间 (用于超时检测) */

    /* === TDMA 微时隙配置 === */
    bool     tdma_enabled;       /* 是否启用分时发送 */
    uint32_t tdma_period_us;     /* 发送总周期 (如 5000us = 200Hz) */
    uint32_t tdma_offset_us;     /* 本节点时隙起始偏移量 */
    uint32_t tdma_window_us;     /* 允许发送的窗口大小 */
} time_sync_state_t;

/**
 * @brief 初始化时间同步模块 (节点端)
 */
void time_sync_init(time_sync_state_t *state);

/**
 * @brief 处理收到的同步包 (在 ESP-NOW recv 回调中调用)
 *
 * @param pkt       收到的同步包
 * @param pkt_len   包长度
 * @return true 本次包已处理, false 不是同步包或忽略
 */
bool time_sync_handle_rx(time_sync_state_t *state,
                          const uint8_t *pkt, int pkt_len);

/**
 * @brief 生成 SYNC_REPLY 回复包 (收到 SYNC_START 后调用)
 *
 * @param[out] reply_buf  输出缓冲区 (至少 sizeof(timesync_packet_t))
 * @param      reply_len  输出包长度
 * @param      src_pkt    收到的 SYNC_START 包
 * @param      node_id    本节点 ID
 */
void time_sync_gen_reply(uint8_t *reply_buf, int *reply_len,
                          const timesync_packet_t *src_pkt, uint8_t node_id);

/**
 * @brief 获取同步后的当前时间 (μs)
 *
 * 如果已同步: 返回 local_time + offset
 * 如果未同步: 返回 local_time (无修正)
 */
int64_t time_sync_get_time(time_sync_state_t *state);

/**
 * @brief 同步是否有效 (距上次同步 < 5 秒)
 */
bool time_sync_is_valid(const time_sync_state_t *state);

/**
 * @brief 设置 TDMA 时隙参数
 */
void time_sync_set_tdma(time_sync_state_t *state, bool enable,
                         uint32_t period, uint32_t offset, uint32_t window);

/**
 * @brief 判断当前是否在本节点的 TDMA 时隙内
 *
 * 利用全局同步时间取模, 计算当前相位是否落在本节点窗口内。
 * 未启用或未同步时退化为立即发送。
 */
bool time_sync_is_my_slot(time_sync_state_t *state);

/**
 * @brief 计算距离下一个本节点 TDMA 时隙的微秒数
 *
 * 返回值:
 *   > 0  距离下一个时隙的等待时间 (μs)
 *   ≤ 0  已经在时隙内或未同步 (应立即发送)
 *
 * 用于混合调度: 粗调 vTaskDelay (>2ms) + 微调 esp_rom_delay_us (<2ms)
 */
int64_t time_sync_us_until_next_slot(time_sync_state_t *state);

/* ============================================================
 *  主机端 API (RK3566/PC, 仅做参考)
 * ============================================================ */
typedef struct {
    uint32_t seq;               /* 当前序列号 */
    int64_t  last_send_us;      /* 上次发送 SYNC_START 的本地时间 */
    int64_t  rtt_us;            /* 测量的 RTT (μs) */
} time_sync_host_t;

/**
 * @brief 生成 SYNC_START 包 (主机每秒广播一次)
 */
void time_sync_host_gen_start(time_sync_host_t *host,
                               uint8_t *buf, int *buf_len, uint8_t node_id);

/**
 * @brief 处理 SYNC_REPLY, 生成 SYNC_APPLY
 */
bool time_sync_host_handle_reply(time_sync_host_t *host,
                                  const timesync_packet_t *reply,
                                  uint8_t *apply_buf, int *apply_len);

/**
 * @brief 生成心跳包 (维持同步)
 */
void time_sync_host_gen_heartbeat(time_sync_host_t *host,
                                   uint8_t *buf, int *buf_len);

#ifdef __cplusplus
}
#endif

#endif /* TIME_SYNC_H */
