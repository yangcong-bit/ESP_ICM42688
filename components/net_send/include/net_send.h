/**
 * @file net_send.h
 * @brief WiFi 连接 + UDP/HTTP 数据发送
 *
 * 支持两种模式:
 *   1. UDP  — 轻量级, 适合高频 IMU 数据流 (默认)
 *   2. HTTP POST — 适合 JSON 格式上报
 */

#ifndef NET_SEND_H
#define NET_SEND_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================
 *  WiFi STA 配置
 * ============================================================ */
typedef struct {
    const char *ssid;         /* WiFi 名称 */
    const char *password;     /* WiFi 密码 */
    int         max_retry;    /* 最大重连次数 (默认 5) */
} net_wifi_cfg_t;

/* ============================================================
 *  UDP 发送配置
 * ============================================================ */
typedef struct {
    const char *host;         /* 目标 IP 地址 */
    uint16_t    port;         /* 目标端口号 */
} net_udp_cfg_t;

/* ============================================================
 *  HTTP POST 配置
 * ============================================================ */
typedef struct {
    const char *url;          /* 完整 URL, 如 "http://192.168.1.100:8080/api/imu" */
    int         timeout_ms;   /* 超时时间 (ms), 默认 3000 */
} net_http_cfg_t;

/* ============================================================
 *  发送协议枚举
 * ============================================================ */
typedef enum {
    NET_PROTO_UDP,
    NET_PROTO_HTTP,
} net_proto_t;

/* ============================================================
 *  IMU 数据包 (用于序列化)
 * ============================================================ */
typedef struct {
    float    accel[3];        /* 加速度 (g) */
    float    gyro[3];         /* 陀螺仪 (dps) */
    float    temp;            /* 温度 (°C) */
    float    quat[4];         /* 四元数 [w, x, y, z] */
    float    euler[3];        /* 欧拉角 (度) [roll, pitch, yaw] */
    uint64_t timestamp_us;    /* 时间戳 (μs) */
} net_imu_packet_t;

/* ============================================================
 *  API
 * ============================================================ */

/**
 * @brief 初始化 WiFi (STA 模式)
 * @param cfg WiFi 配置 (SSID, 密码)
 * @return true 连接成功, false 失败
 */
bool net_wifi_init(const net_wifi_cfg_t *cfg);

/**
 * @brief 获取当前 WiFi 连接状态
 */
bool net_wifi_is_connected(void);

/**
 * @brief 获取本机 IP 地址字符串
 */
const char *net_wifi_get_ip(void);

/**
 * @brief 初始化 UDP 发送
 * @param cfg 目标服务器 IP + 端口
 * @return true 成功
 */
bool net_udp_init(const net_udp_cfg_t *cfg);

/**
 * @brief 通过 UDP 发送 IMU 数据包 (二进制, 高效)
 *
 * 数据格式 (48 bytes):
 *   [0-11]   accel float[3]
 *   [12-23]  gyro  float[3]
 *   [24-27]  temp  float
 *   [28-43]  quat  float[4]
 *   [44-55]  euler float[3]
 *   [56-63]  timestamp uint64
 */
bool net_udp_send_imu(const net_imu_packet_t *pkt);

/**
 * @brief 通过 UDP 发送原始字符串
 */
bool net_udp_send_raw(const char *data, size_t len);

/**
 * @brief 初始化 HTTP POST 发送
 * @param cfg HTTP 目标 URL
 * @return true 成功
 */
bool net_http_init(const net_http_cfg_t *cfg);

/**
 * @brief 通过 HTTP POST 发送 IMU 数据 (JSON 格式)
 *
 * POST body 示例:
 *   {"a":[0.01,0.02,9.78],"g":[0.1,-0.3,0.05],"t":25.3,
 *    "q":[0.999,0.01,0.02,0.005],"e":[1.2,-0.8,45.3],"ts":123456}
 */
bool net_http_send_imu(const net_imu_packet_t *pkt);

/**
 * @brief 通过 HTTP POST 发送自定义 JSON 字符串
 */
bool net_http_send_json(const char *json_str);

/**
 * @brief 关闭 WiFi
 */
void net_deinit(void);

/* ============================================================
 *  聚合数据包 (10帧合1包, 降低物理帧头开销)
 * ============================================================ */
#define NET_AGGREGATE_FRAMES  10

/* 单帧紧凑数据 */
typedef struct __attribute__((packed)) {
    float    accel[3];        /* 加速度 (g) */
    float    gyro[3];         /* 陀螺仪 (dps) */
    float    quat[4];         /* 四元数 [w, x, y, z] */
    float    euler[3];        /* 欧拉角 [roll, pitch, yaw] */
    float    temp;            /* 温度 (°C) */
    uint64_t timestamp_us;    /* 全局同步时间戳 (μs) */
} net_frame_t;  /* 52 bytes */

/* 聚合包: header + N帧 */
typedef struct __attribute__((packed)) {
    uint8_t  magic[4];        /* {'I','M','U','A'} */
    uint8_t  node_id;         /* 节点 ID */
    uint8_t  frame_count;     /* 本包包含的帧数 (1~10) */
    uint16_t reserved;        /* 保留/对齐 */
    net_frame_t frames[NET_AGGREGATE_FRAMES];
} net_aggregated_packet_t;

/* ============================================================
 *  API — ESP-NOW 统计
 * ============================================================ */

/**
 * @brief 获取 ESP-NOW 发送成功次数
 */
uint32_t net_espnow_get_send_ok(void);

/**
 * @brief 获取 ESP-NOW 发送失败次数
 */
uint32_t net_espnow_get_send_fail(void);

/* ============================================================
 *  API — 全局时间同步
 * ============================================================ */

/**
 * @brief 初始化时间同步模块
 */
void net_time_sync_init(void);

/**
 * @brief 设置本节点 ID (用于时间同步回复)
 */
void net_set_node_id(uint8_t id);

/**
 * @brief 获取全局同步时间 (μs)
 *
 * 已同步时返回 host_time + offset, 否则返回本地时间
 */
int64_t net_get_synced_time(void);

/**
 * @brief 时间同步是否有效 (5 秒内有同步)
 */
bool net_time_sync_valid(void);

/**
 * @brief 获取同步次数
 */
uint32_t net_time_sync_count(void);

/* ============================================================
 *  API — 聚合发送
 * ============================================================ */

/**
 * @brief 发送聚合 IMU 数据包 (10帧合1)
 */
bool net_udp_send_aggregated(const net_aggregated_packet_t *agg);

#ifdef __cplusplus
}
#endif

#endif /* NET_SEND_H */
