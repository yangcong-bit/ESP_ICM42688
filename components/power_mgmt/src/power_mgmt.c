/**
 * @file power_mgmt.c
 * @brief 极致电源管理: DFS + WoM 休眠 + ULP 低电量保护
 *
 * 状态机:
 *   Active (240MHz) → Idle (80MHz) → WoM Sleep (Deep Sleep, IMU唤醒)
 *                                       ↓
 *                                  Dead Zone (ULP接管, 充电唤醒)
 */

#include "power_mgmt.h"
#include "esp_log.h"
#include "esp_pm.h"
#include "esp_sleep.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "ulp_riscv.h"
#include "rom/ets_sys.h"
#include <math.h>
#include <string.h>

static const char *TAG = "power_mgmt";

/* ============================================================
 *  内部辅助: DFS 切换
 * ============================================================ */
static void set_cpu_freq(uint32_t mhz)
{
#if CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ_240
    /* 只有配置了 240MHz 才能动态切换 */
    rtc_cpu_freq_config_t config;
    rtc_clk_cpu_freq_get_config(&config);

    uint32_t cur_mhz = config.freq_mhz;
    if (cur_mhz == mhz) return;

    rtc_cpu_freq_config_t target = config;
    target.freq_mhz = mhz;
    /* 根据频率选择对应的分频器 */
    if (mhz >= 240) {
        target.div = RTC_CPU_FREQ_SRC_240M;
        target.freq_mhz = 240;
    } else if (mhz >= 160) {
        target.div = RTC_CPU_FREQ_SRC_160M;
        target.freq_mhz = 160;
    } else {
        target.div = RTC_CPU_FREQ_SRC_80M;
        target.freq_mhz = 80;
    }

    rtc_clk_cpu_freq_set_config(&target);
    ESP_LOGD(TAG, "CPU freq: %lu → %lu MHz", (unsigned long)cur_mhz, (unsigned long)target.freq_mhz);
#endif
}

/* ============================================================
 *  API — 初始化
 * ============================================================ */
void pm_init(pm_ctx_t *pm, const pm_cfg_t *cfg)
{
    if (!pm) return;
    memset(pm, 0, sizeof(*pm));

    if (cfg) {
        pm->cfg = *cfg;
    } else {
        /* 默认配置 */
        pm->cfg.active_freq_mhz     = 240;
        pm->cfg.idle_freq_mhz       = 80;
        pm->cfg.dead_zone_voltage   = 3.3f;
        pm->cfg.wake_voltage        = 3.5f;
        pm->cfg.sleep_cfg.accel_threshold_g   = 0.05f;
        pm->cfg.sleep_cfg.gyro_threshold_dps  = 1.0f;
        pm->cfg.sleep_cfg.sleep_delay_ms      = 30000;  /* 30秒静止 */
        pm->cfg.sleep_cfg.wake_gpio           = 0;      /* 未配置 */
    }

    pm->still_threshold = pm->cfg.sleep_cfg.sleep_delay_ms;  /* 1ms/帧 → ms 直接对比 */
    pm->state = PM_ACTIVE;

    ESP_LOGI(TAG, "Power Mgmt: DFS %lu→%lu MHz, DeadZone=%.1fV, Wake=%.1fV, Stillness=%lums",
             (unsigned long)pm->cfg.active_freq_mhz,
             (unsigned long)pm->cfg.idle_freq_mhz,
             pm->cfg.dead_zone_voltage,
             pm->cfg.wake_voltage,
             (unsigned long)pm->cfg.sleep_cfg.sleep_delay_ms);
}

/* ============================================================
 *  API — DFS 切换
 * ============================================================ */
void pm_enter_active(pm_ctx_t *pm)
{
    if (!pm || pm->state == PM_ACTIVE) return;
    set_cpu_freq(pm->cfg.active_freq_mhz);
    pm->state = PM_ACTIVE;
    pm->dfs_switch_count++;
}

void pm_enter_idle(pm_ctx_t *pm)
{
    if (!pm || pm->state == PM_IDLE) return;
    set_cpu_freq(pm->cfg.idle_freq_mhz);
    pm->state = PM_IDLE;
    pm->dfs_switch_count++;
}

/* ============================================================
 *  API — 静止检测
 * ============================================================ */
bool pm_check_stillness(pm_ctx_t *pm, const float accel_g[3], const float gyro_dps[3])
{
    if (!pm) return false;

    /* 计算加速度变化量 (相对于 1g 重力) */
    float a_norm = sqrtf(accel_g[0]*accel_g[0] + accel_g[1]*accel_g[1] + accel_g[2]*accel_g[2]);
    float a_deviation = fabsf(a_norm - 1.0f);  /* 偏离 1g 的量 */

    /* 计算陀螺仪变化量 */
    float g_norm = sqrtf(gyro_dps[0]*gyro_dps[0] + gyro_dps[1]*gyro_dps[1] + gyro_dps[2]*gyro_dps[2]);

    /* 静止判定 */
    bool is_still = (a_deviation < pm->cfg.sleep_cfg.accel_threshold_g) &&
                    (g_norm < pm->cfg.sleep_cfg.gyro_threshold_dps);

    if (is_still) {
        pm->still_count++;
        if (pm->still_count >= pm->still_threshold) {
            ESP_LOGW(TAG, "静止检测触发! 持续 %lu ms > 阈值 %lu ms, 准备进入 WoM 休眠",
                     (unsigned long)(pm->still_count),
                     (unsigned long)pm->still_threshold);
            return true;  /* 触发休眠 */
        }
    } else {
        /* 一旦有运动, 重置计数器 */
        if (pm->still_count > 0) {
            pm->still_count = 0;
        }
    }
    return false;
}

/* ============================================================
 *  API — WoM Deep Sleep
 * ============================================================ */
void pm_enter_wom_sleep(pm_ctx_t *pm)
{
    if (!pm) return;

    ESP_LOGW(TAG, "=== 进入 WoM Deep Sleep ===");

    /* 1. 关闭 OLED */
    if (pm->cfg.oled_pwr_gpio >= 0) gpio_set_level(pm->cfg.oled_pwr_gpio, 0);
    if (pm->cfg.oled_res_gpio >= 0) gpio_set_level(pm->cfg.oled_res_gpio, 0);

    /* 2. 通过 SPI 让 IMU 进入 WoM 模式 (需要外部调用) */
    /* 此处预留接口, 实际由主核在调用前完成 IMU WoM 配置 */

    /* 3. 配置 RTC GPIO 唤醒 (IMU INT1 引脚) */
    if (pm->cfg.sleep_cfg.wake_gpio > 0) {
        esp_sleep_enable_ext0_wakeup(pm->cfg.sleep_cfg.wake_gpio, 0);  /* 低电平唤醒 */
    }

    /* 4. 禁用 WiFi (ESP-NOW 在深睡时自动关闭) */

    /* 5. 进入深睡 */
    pm->state = PM_WOM_SLEEP;
    ESP_LOGW(TAG, "进入 WoM Deep Sleep, 等待 IMU 动作唤醒...");
    ESP_LOGW(TAG, "wake_gpio=%lu", (unsigned long)pm->cfg.sleep_cfg.wake_gpio);

    esp_deep_sleep_start();
    /* ---- 此处永远不会返回, 除非被 IMU INT1 唤醒 ---- */
}

/* ============================================================
 *  API — 死区 Deep Sleep (ULP 接管)
 * ============================================================ */
void pm_enter_dead_zone(pm_ctx_t *pm)
{
    if (!pm) return;

    ESP_LOGE(TAG, "============================================");
    ESP_LOGE(TAG, "!!! 电池死区! 进入 ULP 接管休眠 !!!");
    ESP_LOGE(TAG, "当前电压 < %.1fV, ULP 将以 %.0fHz 监控", pm->cfg.dead_zone_voltage, 2.0f);
    ESP_LOGE(TAG, "只有充电至 >= %.1fV 时才会被唤醒", pm->cfg.wake_voltage);
    ESP_LOGE(TAG, "============================================");

    /* 1. 关闭所有外设 */
    if (pm->cfg.oled_pwr_gpio >= 0) gpio_set_level(pm->cfg.oled_pwr_gpio, 0);
    if (pm->cfg.oled_res_gpio >= 0) gpio_set_level(pm->cfg.oled_res_gpio, 0);
    if (pm->cfg.ldo_en1_gpio >= 0)  gpio_set_level(pm->cfg.ldo_en1_gpio, 0);   /* IMU-A LDO 关 */
    if (pm->cfg.ldo_en2_gpio >= 0)  gpio_set_level(pm->cfg.ldo_en2_gpio, 0);   /* IMU-B LDO 关 */

    /* 2. 禁用 IMU WoM 唤醒源 (电池没电时不需要动作唤醒) */
    esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_ALL);

    /* 3. 配置 ULP Timer (每 500ms 唤醒一次 = 2Hz) */
    esp_sleep_enable_timer_wakeup(500000);  /* 500ms */

    /* 4. 加载 ULP 程序 (已编译的二进制) */
    pm->ulp_active = true;

    /* 5. 设置 ULP ADC 阈值 (共享 RTC 内存) */
    extern volatile uint32_t ulp_adc_threshold_3_5v;
    ulp_adc_threshold_3_5v = pm->adc_threshold_3_5v;

    /* 6. 启动 ULP 协处理器 */
    ulp_riscv_run();

    /* 7. 进入无限期深睡 */
    pm->state = PM_DEAD_ZONE_SLEEP;
    ESP_LOGW(TAG, "启动 ULP 监控, 主核进入无限期 Deep Sleep...");

    esp_deep_sleep_start();
    /* ---- 此处永远不会返回, 除非 ULP 唤醒 ---- */
}

/* ============================================================
 *  API — 唤醒判断
 * ============================================================ */
bool pm_is_wakeup_from_ulp(void)
{
    /* 检查唤醒原因 */
    esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();

    if (cause == ESP_SLEEP_WAKEUP_ULP) {
        ESP_LOGI(TAG, "唤醒源: ULP (充电恢复)");
        return true;
    } else if (cause == ESP_SLEEP_WAKEUP_EXT0) {
        ESP_LOGI(TAG, "唤醒源: EXT0 (IMU WoM 动作)");
        return false;
    } else {
        ESP_LOGW(TAG, "唤醒源: %d (其他)", cause);
        return false;
    }
}

/* ============================================================
 *  API — ULP 初始化
 * ============================================================ */
void pm_ulp_init(pm_ctx_t *pm, int adc_raw_threshold)
{
    if (!pm) return;

    pm->adc_threshold_3_5v = (uint32_t)adc_raw_threshold;

    ESP_LOGI(TAG, "ULP 初始化: ADC阈值=%lu (对应 %.1fV)",
             (unsigned long)pm->adc_threshold_3_5v, pm->cfg.wake_voltage);
}

/* ============================================================
 *  API — 状态名称
 * ============================================================ */
const char *pm_state_name(pm_state_t state)
{
    switch (state) {
        case PM_ACTIVE:          return "ACTIVE(240MHz)";
        case PM_IDLE:            return "IDLE(80MHz)";
        case PM_WOM_SLEEP:       return "WOM_SLEEP";
        case PM_DEAD_ZONE_SLEEP: return "DEAD_ZONE";
        default:                 return "UNKNOWN";
    }
}
