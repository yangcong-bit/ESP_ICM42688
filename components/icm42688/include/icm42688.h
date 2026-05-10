/**
 * @file icm42688.h
 * @brief ICM-42688-P SPI 驱动 — 公共 API
 *
 * 适用于 ESP32-S3 + ESP-IDF 5.x
 * 使用 hardware SPI master, 支持 DMA
 */

#ifndef ICM42688_H
#define ICM42688_H

#include <stdint.h>
#include <stdbool.h>
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/event_groups.h"
#include "icm42688_reg.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================
 *  SPI 引脚 & 设备配置
 * ============================================================ */
typedef struct {
    int         spi_host;       /* SPI2_HOST / SPI3_HOST */
    int         pin_sclk;
    int         pin_mosi;
    int         pin_miso;
    int         pin_cs;
    int         pin_int;        /* INT1 中断引脚, -1 = 不使用中断 */
    int         clock_speed_hz;/* SPI 时钟, 建议 ≤10MHz */
} icm42688_spi_cfg_t;

/* ============================================================
 *  传感器配置
 * ============================================================ */
typedef struct {
    icm42688_accel_fs_t  accel_fs;    /* 加速度计量程 */
    icm42688_gyro_fs_t   gyro_fs;     /* 陀螺仪量程 */
    icm42688_odr_t       accel_odr;   /* 加速度计 ODR */
    icm42688_odr_t       gyro_odr;    /* 陀螺仪 ODR */
    bool                 enable_fifo; /* 是否启用 FIFO */
} icm42688_cfg_t;

/* ============================================================
 *  三轴原始数据 (int16)
 * ============================================================ */
typedef struct {
    int16_t x;
    int16_t y;
    int16_t z;
} icm42688_axis3i16_t;

/* ============================================================
 *  三轴浮点数据 (物理单位)
 * ============================================================ */
typedef struct {
    float x;  /* g  或 dps */
    float y;
    float z;
} icm42688_axis3f_t;

/* ============================================================
 *  传感器读数 (完整包)
 * ============================================================ */
typedef struct {
    icm42688_axis3i16_t  accel_raw;   /* 原始加速度 (LSB) */
    icm42688_axis3i16_t  gyro_raw;    /* 原始陀螺仪 (LSB) */
    float                temp_raw;    /* 原始温度 (°C) */
    icm42688_axis3f_t    accel_g;     /* 加速度 (g) */
    icm42688_axis3f_t    gyro_dps;    /* 陀螺仪 (dps) */
    uint64_t             timestamp_us;/* 获取时间 (μs) */
} icm42688_reading_t;

/* ============================================================
 *  设备句柄
 * ============================================================ */
typedef struct {
    spi_device_handle_t  spi_dev;
    icm42688_cfg_t       cfg;
    icm42688_axis3f_t    accel_bias;  /* 加速度偏移校准 (g) */
    icm42688_axis3f_t    gyro_bias;   /* 陀螺仪偏移校准 (dps) */
    int                  int_gpio;    /* 中断引脚号, -1 = 未配置 */
    SemaphoreHandle_t    int_sem;     /* Data Ready 二值信号量 */
    EventGroupHandle_t   int_evtgrp;  /* Data Ready 事件组 (双路同步) */
    volatile uint32_t    int_count;   /* 中断触发计数 (诊断用) */
    uint8_t             *dma_tx_buf;  /* 预分配 DMA TX 缓冲 (init时分配) */
    uint8_t             *dma_rx_buf;  /* 预分配 DMA RX 缓冲 (init时分配) */
    bool                 initialized;
} icm42688_dev_t;

/* ============================================================
 *  返回码
 * ============================================================ */
typedef enum {
    ICM42688_OK = 0,
    ICM42688_ERR_SPI_INIT,
    ICM42688_ERR_WHO_AM_I,
    ICM42688_ERR_RESET,
    ICM42688_ERR_CONFIG,
    ICM42688_ERR_TIMEOUT,
    ICM42688_ERR_NOT_INIT,
} icm42688_err_t;

/* ============================================================
 *  API — 初始化 & 配置
 * ============================================================ */

/**
 * @brief 初始化 ICM42688 设备
 * @param dev       设备句柄 (调用者分配)
 * @param spi_cfg   SPI 引脚 & 时钟配置
 * @param sensor_cfg 传感器量程 / ODR 配置, NULL = 默认值
 * @return ICM42688_OK 成功
 */
icm42688_err_t icm42688_init(icm42688_dev_t *dev,
                             const icm42688_spi_cfg_t *spi_cfg,
                             const icm42688_cfg_t *sensor_cfg);

/**
 * @brief 软复位并重新初始化
 */
icm42688_err_t icm42688_reset(icm42688_dev_t *dev);

/**
 * @brief 关闭设备 (进入最低功耗)
 */
void icm42688_deinit(icm42688_dev_t *dev);

/* ============================================================
 *  API — 数据读取 (polling)
 * ============================================================ */

/**
 * @brief 读取一次完整的传感器数据 (温度+加速度+陀螺仪)
 *
 * 内部按寄存器顺序一次性 burst read 14 bytes:
 *   TEMP_DATA1..0, ACCEL_XYZ(6), GYRO_XYZ(6)
 *
 * @param[out] reading  输出读数
 */
icm42688_err_t icm42688_read_polling(icm42688_dev_t *dev,
                                     icm42688_reading_t *reading);

/**
 * @brief 异步提交 SPI DMA 传输 (非阻塞)
 * 配合 icm42688_wait_read 完成双路 SPI 真正并发:
 *   queue_trans(A) → queue_trans(B) → get_result(A) → get_result(B)
 */
icm42688_err_t icm42688_queue_read(icm42688_dev_t *dev);

/**
 * @brief 等待并收割之前提交的 SPI DMA 传输结果
 */
icm42688_err_t icm42688_wait_read(icm42688_dev_t *dev,
                                    icm42688_reading_t *reading);

/* ============================================================
 *  API — 中断驱动读取
 * ============================================================ */

/**
 * @brief 配置 INT1 引脚为 Data Ready 中断 (GPIO 下降沿)
 *
 * 内部创建二值信号量, 每次 ICM42688 数据就绪时 ISR 释放信号量。
 * 必须在 icm42688_init 之后、读取循环之前调用。
 *
 * @param int_pin  INT1 连接的 ESP32 GPIO 引脚号
 */
icm42688_err_t icm42688_init_interrupt(icm42688_dev_t *dev, int int_pin);

/**
 * @brief 等待 Data Ready 中断 (阻塞, 超时)
 *
 * 内部 xSemaphoreTake, 当 INT1 触发时返回。
 * 推荐在高优先级任务中调用, 替代 vTaskDelay 轮询。
 *
 * @param timeout_ms  超时毫秒, portMAX_DELAY = 永久等待
 * @return ICM42688_OK 收到中断, ICM42688_ERR_TIMEOUT 超时
 */
icm42688_err_t icm42688_wait_drdy(icm42688_dev_t *dev, uint32_t timeout_ms);

/**
 * @brief 获取中断触发次数 (诊断用)
 */
uint32_t icm42688_get_int_count(const icm42688_dev_t *dev);

/**
 * @brief 双路 EventGroup 同时等待 Data Ready
 *
 * 同时等待 dev_a 和 dev_b 的中断事件位,
 * 任一路触发即返回, 实现真正的并发等待。
 * 解决顺序阻塞导致的延迟问题。
 *
 * @param dev_a      IMU-A 设备
 * @param dev_b      IMU-B 设备
 * @param timeout_ms 超时毫秒
 * @return ICM42688_OK 至少一路触发, ICM42688_ERR_TIMEOUT 全部超时
 */
icm42688_err_t icm42688_wait_drdy_group(icm42688_dev_t *dev_a,
                                          icm42688_dev_t *dev_b,
                                          uint32_t timeout_ms);

/* ============================================================
 *  API — 偏移校准
 * ============================================================ */

/**
 * @brief 静态六面校准
 *
 * 在传感器静止时调用，采集若干样本计算零偏偏移
 * @param samples 采样次数 (建议 ≥200)
 */
icm42688_err_t icm42688_calibrate_bias(icm42688_dev_t *dev, uint32_t samples);

/**
 * @brief 获取已存储的偏移
 */
icm42688_axis3f_t icm42688_get_accel_bias(const icm42688_dev_t *dev);
icm42688_axis3f_t icm42688_get_gyro_bias(const icm42688_dev_t *dev);

/**
 * @brief 手动设置偏移
 */
void icm42688_set_accel_bias(icm42688_dev_t *dev, icm42688_axis3f_t bias);
void icm42688_set_gyro_bias(icm42688_dev_t *dev, icm42688_axis3f_t bias);

/* ============================================================
 *  API — 寄存器读写 (高级)
 * ============================================================ */
icm42688_err_t icm42688_read_reg(icm42688_dev_t *dev,
                                  uint8_t reg, uint8_t *val);
icm42688_err_t icm42688_write_reg(icm42688_dev_t *dev,
                                   uint8_t reg, uint8_t val);
icm42688_err_t icm42688_read_regs(icm42688_dev_t *dev,
                                   uint8_t reg, uint8_t *buf, size_t len);

/* ============================================================
 *  辅助 — 量程换算
 * ============================================================ */
float icm42688_accel_fs_to_sensitivity(icm42688_accel_fs_t fs);
float icm42688_gyro_fs_to_sensitivity(icm42688_gyro_fs_t fs);

#ifdef __cplusplus
}
#endif

#endif /* ICM42688_H */
