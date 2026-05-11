/**
 * @file ulp_main.c
 * @brief ULP-RISC-V 协处理器程序: 迟滞低电量监控
 *
 * 运行在 Deep Sleep 期间, 以极低功耗 (约 10μA) 静默读取 RTC ADC,
 * 仅当电池电压恢复到充电阈值以上时才唤醒主核。
 *
 * 编译: 需要 ESP-IDF 的 ULP 构建系统支持
 * 链接: 通过 CMakeLists.txt 的 ulp_embed_txt() 宏自动处理
 */

#include <stdint.h>
#include "ulp_riscv.h"
#include "ulp_riscv_utils.h"
#include "ulp_riscv_adc.h"
#include "hal/adc_types.h"

/* ============================================================
 *  共享变量 (主核校准后写入, ULP 只读)
 * ============================================================ */
volatile uint32_t adc_threshold_3_5v;  /* 3.5V 对应的 ADC 原始值 (由主核计算) */
volatile uint32_t ulp_wake_count;      /* ULP 唤醒次数计数 (诊断用) */
volatile uint32_t ulp_adc_reading;     /* 最近一次 ADC 读数 (调试用) */

/* ============================================================
 *  ULP-RISC-V 主入口
 *
 *  每次 ULP Timer 唤醒后执行一次:
 *    1. 读取 RTC ADC
 *    2. 迟滞判断: 只有电压 >= 3.5V 才唤醒主核
 *    3. 返回 → ULP 自动进入睡眠, 等待下次 Timer 唤醒
 * ============================================================ */
int main(void)
{
    /* 读取 RTC ADC (电池电压通道) */
    uint32_t adc_raw = 0;
    ulp_riscv_adc_read_channel(ADC_UNIT_1, ADC_CHANNEL_7, &adc_raw);

    /* 记录读数 (主核可通过 ULP RTC 内存读取此值进行调试) */
    ulp_adc_reading = adc_raw;

    /* 迟滞判断: 电压 >= 3.5V → 唤醒主核 */
    if (adc_raw >= adc_threshold_3_5v) {
        ulp_wake_count++;
        ulp_riscv_wakeup_main_processor();
    }

    /* 返回 0 → ULP 进入睡眠, 等待 ULP Timer 下一次唤醒 */
    return 0;
}
