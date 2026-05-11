/**
 * @file ulp_main.c
 * @brief ULP-RISC-V: 迟滞低电量监控 (Deep Sleep 期间运行)
 *
 * 编译: idf.py build 自动编译 ULP 程序
 * 使用: 在主核侧通过 ulp_riscv_load() 加载
 */

#include <stdint.h>
#include "ulp_riscv_utils.h"
#include "ulp_riscv_adc.h"

/* 共享变量 (主核写入, ULP 只读) */
volatile uint32_t adc_threshold_3_5v;   /* 3.5V 对应的 ADC 原始值 */
volatile uint32_t ulp_wake_count;       /* 唤醒次数 (诊断) */
volatile uint32_t ulp_adc_reading;      /* 最近 ADC 读数 (调试) */

int main(void)
{
    /* 读取 RTC ADC */
    uint32_t adc_raw = 0;
    ulp_riscv_adc_read_channel(ULP_RISCV_ADC_UNIT_1, ULP_RISCV_ADC_CHANNEL_7, &adc_raw);
    ulp_adc_reading = adc_raw;

    /* 迟滞: 电压 >= 3.5V → 唤醒主核 */
    if (adc_raw >= adc_threshold_3_5v) {
        ulp_wake_count++;
        ulp_riscv_wakeup_main_processor();
    }

    return 0;  /* ULP 睡眠, 等待下次 Timer */
}
