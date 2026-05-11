/**
 * @file ulp_main.c
 * @brief ULP-RISC-V 迟滞低电量监控
 *
 * Deep Sleep 期间以 ~10μA 功耗静默读取 RTC ADC。
 * 硬件连线: 电池分压 → GPIO2 → ADC1_CH1。
 * 仅当电压恢复到 ≥3.5V (充电器接入) 时唤醒主核。
 *
 * 防抖: 连续 3 次采样取平均, 防止浪涌电压误唤醒。
 */

#include <stdint.h>
#include "ulp_riscv.h"
#include "ulp_riscv_utils.h"
#include "ulp_riscv_adc_ulp_core.h"
#include "hal/adc_types.h"

/* 共享变量 (主核写入, ULP 只读) */
volatile uint32_t ulp_adc_threshold_3_5v;  /* 3.5V 对应的 ADC 原始值 */
volatile uint32_t ulp_wake_count;          /* 唤醒次数 (诊断) */
volatile uint32_t ulp_adc_reading;         /* 最近 ADC 读数 (调试) */

int main(void)
{
    /* 连续读取 3 次取平均, 防止充电器浪涌电压误触发 */
    uint32_t adc_sum = 0;
    for (int i = 0; i < 3; i++) {
        adc_sum += ulp_riscv_adc_read_channel(ADC_UNIT_1, ADC_CHANNEL_1);
    }
    uint32_t adc_avg = adc_sum / 3;

    /* 记录读数 */
    ulp_adc_reading = adc_avg;

    /* 迟滞判断: 电压 ≥ 3.5V → 唤醒主核 */
    if (adc_avg >= ulp_adc_threshold_3_5v) {
        ulp_wake_count++;
        ulp_riscv_wakeup_main_processor();
    }

    return 0;  /* ULP 睡眠, 等待 Timer */
}
