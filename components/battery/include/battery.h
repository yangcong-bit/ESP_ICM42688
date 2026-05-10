/**
 * @file battery.h
 * @brief 电池电压检测驱动 (ADC1_CH1, IO2)
 */

#ifndef BATTERY_H
#define BATTERY_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================
 *  硬件参数
 * ============================================================ */
#define BAT_ADC_UNIT        ADC_UNIT_1
#define BAT_ADC_CHANNEL     ADC_CHANNEL_1   /* GPIO2 → ADC1_CH1 */
#define BAT_ATTEN           ADC_ATTEN_DB_12 /* 满量程 ~3100mV */
#define BAT_VDIV_RATIO      2.0f            /* 分压系数: Vbat = Vadc × ratio */
#define BAT_FULL_MV         4200            /* 锂电池满电电压 (mV) */
#define BAT_EMPTY_MV        3300            /* 锂电池截止电压 (mV) */
#define BAT_SAMPLE_COUNT    10              /* 多重采样次数 */

/* ============================================================
 *  返回码
 * ============================================================ */
typedef enum {
    BATTERY_OK = 0,
    BATTERY_ERR_INIT,
    BATTERY_ERR_READ,
} battery_err_t;

/* ============================================================
 *  API
 * ============================================================ */

/**
 * @brief 初始化 ADC 单元、通道及硬件校准
 */
battery_err_t battery_init(void);

/**
 * @brief 读取校准后的 ADC 毫伏值 (引脚电压, 未乘分压系数)
 */
int battery_get_raw_mv(void);

/**
 * @brief 读取电池真实电压 (V, 已乘分压系数, 多重采样滤波)
 */
float battery_get_voltage(void);

/**
 * @brief 估算电池电量百分比 (0~100%, 锂电池放电曲线简易映射)
 */
uint8_t battery_get_percentage(void);

#ifdef __cplusplus
}
#endif

#endif /* BATTERY_H */
