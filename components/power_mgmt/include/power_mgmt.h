/**
 * @file power_mgmt.h
 * @brief 极致电源管理: DFS + WoM 休眠 + ULP 低电量保护
 */

#ifndef POWER_MGMT_H
#define POWER_MGMT_H

#include <stdint.h>
#include <stdbool.h>
#include "soc/soc.h"
#include "esp_pm.h"  /* [Fix 1] esp_pm_lock_handle_t */

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================
 *  电源状态机
 * ============================================================ */
typedef enum {
    PM_ACTIVE,          /* 全速运行: 240MHz, IMU 活跃 */
    PM_IDLE,            /* 降频空闲: 80MHz, 监听模式 */
    PM_WOM_SLEEP,       /* WoM 休眠: IMU 活跃但 CPU 深睡, 被 IMU 动作唤醒 */
    PM_DEAD_ZONE_SLEEP, /* 死区休眠: 电池耗尽, ULP 接管监控 */
} pm_state_t;

/* ============================================================
 *  IMU 静止检测配置
 * ============================================================ */
typedef struct {
    float    accel_threshold_g;   /* 静止判定: 加速度变化量 < 此值 (g) */
    float    gyro_threshold_dps;  /* 静止判定: 陀螺仪变化量 < 此值 (dps) */
    uint32_t sleep_delay_ms;      /* 持续静止多久进入休眠 (ms) */
    uint32_t wake_gpio;           /* WoM 唤醒 GPIO (ICM42688 INT1) */
} pm_sleep_cfg_t;

/* ============================================================
 *  电源管理配置
 * ============================================================ */
typedef struct {
    uint32_t active_freq_mhz;     /* 活动频率: 240MHz */
    uint32_t idle_freq_mhz;       /* 空闲频率: 80MHz */
    float    dead_zone_voltage;   /* 死区电压阈值 (V) */
    float    wake_voltage;        /* ULP 唤醒电压阈值 (V) */
    int      ldo_en1_gpio;        /* IMU-A LDO 使能 GPIO */
    int      ldo_en2_gpio;        /* IMU-B LDO 使能 GPIO */
    int      oled_pwr_gpio;       /* OLED 电源 GPIO */
    int      oled_res_gpio;       /* OLED 复位 GPIO */
    pm_sleep_cfg_t sleep_cfg;     /* 休眠配置 */
} pm_cfg_t;

/* ============================================================
 *  电源管理句柄
 * ============================================================ */
typedef struct {
    pm_state_t      state;
    pm_cfg_t        cfg;

    /* IMU 静止计时器 */
    uint32_t        still_count;      /* 连续静止帧计数 */
    uint32_t        still_threshold;  /* 静止帧阈值 (= sleep_delay_ms / sample_period_ms) */

    /* ULP 配置 */
    uint32_t        adc_threshold_3_5v; /* ULP 用的 ADC 原始阈值 */
    bool            ulp_active;

    /* DFS 统计 */
    uint32_t        dfs_switch_count;

    /* [Fix 1] ESP-IDF PM 锁: 替代手动 rtc_clk_cpu_freq_set_config */
    esp_pm_lock_handle_t cpu_freq_lock;
} pm_ctx_t;

/* ============================================================
 *  API
 * ============================================================ */

/**
 * @brief 初始化电源管理模块
 */
void pm_init(pm_ctx_t *pm, const pm_cfg_t *cfg);

/**
 * @brief 切换到活动模式 (240MHz, 用于 ESKF 解算 + TDMA 时隙发送)
 */
void pm_enter_active(pm_ctx_t *pm);

/**
 * @brief 切换到空闲模式 (80MHz, 用于时隙外监听)
 */
void pm_enter_idle(pm_ctx_t *pm);

/**
 * @brief 检查 IMU 数据是否静止, 并在持续静止时触发休眠
 * @param accel_g   当前加速度 (g)
 * @param gyro_dps  当前陀螺仪 (dps)
 * @return true 如果触发了休眠进入 (调用者应立即退出循环)
 */
bool pm_check_stillness(pm_ctx_t *pm, const float accel_g[3], const float gyro_dps[3]);

/**
 * @brief 进入 WoM Deep Sleep (IMU 动作唤醒)
 * 会通过 SPI 下发 WoM 指令给 IMU, 然后配置 RTC GPIO 并深睡
 */
void pm_enter_wom_sleep(pm_ctx_t *pm);

/**
 * @brief 进入死区 Deep Sleep (电池耗尽, ULP 接管)
 * 断网 → 关 OLED → 关 IMU WoM → 配置 ULP Timer → 深睡
 */
void pm_enter_dead_zone(pm_ctx_t *pm);

/**
 * @brief 主核唤醒后调用: 判断是 WoM 唤醒还是 ULP 充电唤醒
 * @return true = ULP 充电唤醒 (死区恢复), false = WoM 唤醒
 */
bool pm_is_wakeup_from_ulp(void);

/**
 * @brief 初始化 ULP 协处理器 (配置 Timer, 校准 ADC 阈值)
 */
void pm_ulp_init(pm_ctx_t *pm, int adc_raw_threshold);

/**
 * @brief 获取当前电源状态名称
 */
const char *pm_state_name(pm_state_t state);

#ifdef __cplusplus
}
#endif

#endif /* POWER_MGMT_H */
