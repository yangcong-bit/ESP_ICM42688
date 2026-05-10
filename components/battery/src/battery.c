/**
 * @file battery.c
 * @brief 电池电压检测 (ESP-IDF 5.x adc_oneshot API)
 */

#include "battery.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_log.h"
#include <string.h>

static const char *TAG = "battery";

/* 静态句柄 */
static adc_oneshot_unit_handle_t s_adc_handle = NULL;
static adc_cali_handle_t        s_cali_handle = NULL;
static bool s_initialized = false;

/* ============================================================
 *  ADC 校准 (Curve Fitting)
 * ============================================================ */
static bool do_calibration(adc_unit_t unit, adc_channel_t channel,
                           adc_atten_t atten, adc_cali_handle_t *out_handle)
{
    adc_cali_handle_t handle = NULL;
    esp_err_t ret = ESP_OK;

    /* 优先尝试 Curve Fitting (精度更高) */
#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    adc_cali_curve_fitting_config_t cali_config = {
        .unit_id  = unit,
        .chan     = channel,
        .atten   = atten,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    ret = adc_cali_create_scheme_curve_fitting(&cali_config, &handle);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "ADC calibration: Curve Fitting OK");
        *out_handle = handle;
        return true;
    }
#endif

    /* 降级: Line Fitting */
#if ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    adc_cali_line_fitting_config_t line_config = {
        .unit_id  = unit,
        .atten   = atten,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    ret = adc_cali_create_scheme_line_fitting(&line_config, &handle);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "ADC calibration: Line Fitting OK");
        *out_handle = handle;
        return true;
    }
#endif

    ESP_LOGW(TAG, "ADC calibration failed, raw values only");
    *out_handle = NULL;
    return false;
}

/* ============================================================
 *  API — 初始化
 * ============================================================ */
battery_err_t battery_init(void)
{
    ESP_LOGI(TAG, "电池 ADC 初始化 (ADC1_CH1, IO2)");

    /* 1. 创建 ADC oneshot 单元 */
    adc_oneshot_unit_init_cfg_t init_cfg = {
        .unit_id = BAT_ADC_UNIT,
    };
    esp_err_t ret = adc_oneshot_new_unit(&init_cfg, &s_adc_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "ADC oneshot init failed: %s", esp_err_to_name(ret));
        return BATTERY_ERR_INIT;
    }

    /* 2. 配置通道 */
    adc_oneshot_chan_cfg_t chan_cfg = {
        .atten    = BAT_ATTEN,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    ret = adc_oneshot_config_channel(s_adc_handle, BAT_ADC_CHANNEL, &chan_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "ADC channel config failed: %s", esp_err_to_name(ret));
        return BATTERY_ERR_INIT;
    }

    /* 3. 硬件校准 */
    do_calibration(BAT_ADC_UNIT, BAT_ADC_CHANNEL, BAT_ATTEN, &s_cali_handle);

    s_initialized = true;
    ESP_LOGI(TAG, "电池 ADC 初始化完成 (分压系数: %.1f)", BAT_VDIV_RATIO);
    return BATTERY_OK;
}

/* ============================================================
 *  API — 读取原始 mV
 * ============================================================ */
int battery_get_raw_mv(void)
{
    if (!s_initialized || !s_adc_handle) return 0;

    int raw = 0;
    esp_err_t ret = adc_oneshot_read(s_adc_handle, BAT_ADC_CHANNEL, &raw);
    if (ret != ESP_OK) return 0;

    /* 校准转换: raw → mV */
    if (s_cali_handle) {
        int voltage_mv = 0;
        ret = adc_cali_raw_to_voltage(s_cali_handle, raw, &voltage_mv);
        if (ret == ESP_OK) return voltage_mv;
    }

    /* 降级: 手动计算 (12-bit, 3100mV 满量程) */
    return raw * 3100 / 4095;
}

/* ============================================================
 *  API — 读取电池电压 (多重采样滤波)
 * ============================================================ */
float battery_get_voltage(void)
{
    int32_t sum_mv = 0;
    for (int i = 0; i < BAT_SAMPLE_COUNT; i++) {
        sum_mv += battery_get_raw_mv();
    }
    float pin_mv = (float)sum_mv / BAT_SAMPLE_COUNT;
    return pin_mv * BAT_VDIV_RATIO / 1000.0f;  /* mV → V, 乘分压系数 */
}

/* ============================================================
 *  API — 电池电量百分比
 *
 *  简易锂电池放电曲线映射:
 *    4.2V = 100%
 *    3.95V = 75%
 *    3.7V  = 50%
 *    3.5V  = 25%
 *    3.3V  = 0%
 * ============================================================ */
uint8_t battery_get_percentage(void)
{
    float v = battery_get_voltage();
    float mv = v * 1000.0f;

    if (mv >= BAT_FULL_MV)  return 100;
    if (mv <= BAT_EMPTY_MV) return 0;

    /* 线性插值: 3300~4200mV → 0~100% */
    float pct = (mv - BAT_EMPTY_MV) / (BAT_FULL_MV - BAT_EMPTY_MV) * 100.0f;

    /* 简单非线性修正: 锂电池中间段平坦, 两端陡峭 */
    if (pct > 80.0f) {
        /* 80~100%: 缓慢下降 */
        pct = 80.0f + (pct - 80.0f) * 0.5f;
    } else if (pct < 20.0f) {
        /* 0~20%: 快速下降 */
        pct = pct * 0.8f;
    }

    if (pct > 100.0f) pct = 100.0f;
    if (pct < 0.0f) pct = 0.0f;

    return (uint8_t)pct;
}
