/**
 * @file icm42688.c
 * @brief ICM-42688-P SPI 驱动实现
 */

#include "icm42688.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/portmacro.h"
#include "esp_log.h"
#include "esp_heap_caps.h"
#include "esp_timer.h"
#include "driver/gpio.h"
#include <string.h>
#include <math.h>

static const char *TAG = "icm42688";

/* DMA 缓冲大小 (1 byte cmd + 14 bytes data = 15, 对齐到 32) */
#define DMA_BUF_SIZE  32

/* SPI 总线初始化跟踪 (每个 host 只初始化一次) */
#define SPI_HOST_MAX  3
static bool s_spi_bus_inited[SPI_HOST_MAX] = {false};

/* ============================================================
 *  内部辅助
 * ============================================================ */

#define ICM42688_TIMEOUT_MS   100
#define ICM42688_STACK_SIZE   1024

static icm42688_err_t spi_write_reg(icm42688_dev_t *dev, uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = { reg | ICM42688_SPI_WRITE, val };
    spi_transaction_t t = {
        .length    = 2 * 8,
        .tx_buffer = buf,
    };
    esp_err_t ret = spi_device_polling_transmit(dev->spi_dev, &t);
    return (ret == ESP_OK) ? ICM42688_OK : ICM42688_ERR_CONFIG;
}

static icm42688_err_t spi_read_reg(icm42688_dev_t *dev, uint8_t reg, uint8_t *val)
{
    uint8_t cmd = reg | ICM42688_SPI_READ;
    uint8_t dummy = 0;
    spi_transaction_t t = {
        .length    = 2 * 8,
        .tx_buffer = &cmd,
        .rx_buffer = &dummy,
    };
    esp_err_t ret = spi_device_polling_transmit(dev->spi_dev, &t);
    /* SPI read: first byte is garbage, real data 在 byte[1] */
    *val = dummy;   /* 实际收到的数据 */
    return (ret == ESP_OK) ? ICM42688_OK : ICM42688_ERR_CONFIG;
}

/* burst read: 发送 reg 地址, 连续接收 len 字节
 * 复用 dev 预分配的 DMA 缓冲, 零 malloc
 */
static icm42688_err_t spi_read_burst(icm42688_dev_t *dev,
                                      uint8_t reg, uint8_t *buf, size_t len)
{
    if (!dev->dma_tx_buf || !dev->dma_rx_buf) return ICM42688_ERR_CONFIG;

    uint8_t cmd = reg | ICM42688_SPI_READ;
    size_t total = len + 1;
    if (total > DMA_BUF_SIZE) return ICM42688_ERR_CONFIG;

    dev->dma_tx_buf[0] = cmd;
    memset(dev->dma_tx_buf + 1, 0, len);

    spi_transaction_t t = {
        .length    = total * 8,
        .tx_buffer = dev->dma_tx_buf,
        .rx_buffer = dev->dma_rx_buf,
    };
    esp_err_t ret = spi_device_queue_trans(dev->spi_dev, &t, portMAX_DELAY);
    if (ret != ESP_OK) return ICM42688_ERR_CONFIG;
    spi_transaction_t *ret_trans = NULL;
    ret = spi_device_get_trans_result(dev->spi_dev, &ret_trans, portMAX_DELAY);
    if (ret == ESP_OK) {
        memcpy(buf, dev->dma_rx_buf + 1, len);
    }
    return (ret == ESP_OK) ? ICM42688_OK : ICM42688_ERR_CONFIG;
}

/* 异步提交 SPI DMA 传输 (非阻塞, 仅提交不等结果) */
static icm42688_err_t spi_queue_burst(icm42688_dev_t *dev, uint8_t reg, size_t len)
{
    if (!dev->dma_tx_buf || !dev->dma_rx_buf) return ICM42688_ERR_CONFIG;
    uint8_t cmd = reg | ICM42688_SPI_READ;
    size_t total = len + 1;
    if (total > DMA_BUF_SIZE) return ICM42688_ERR_CONFIG;
    dev->dma_tx_buf[0] = cmd;
    memset(dev->dma_tx_buf + 1, 0, len);
    spi_transaction_t t = {
        .length    = total * 8,
        .tx_buffer = dev->dma_tx_buf,
        .rx_buffer = dev->dma_rx_buf,
    };
    return spi_device_queue_trans(dev->spi_dev, &t, portMAX_DELAY) == ESP_OK
           ? ICM42688_OK : ICM42688_ERR_CONFIG;
}

/* 等待并收割之前提交的 SPI DMA 结果 */
static icm42688_err_t spi_collect_burst(icm42688_dev_t *dev, uint8_t *buf, size_t len)
{
    spi_transaction_t *ret_trans = NULL;
    esp_err_t ret = spi_device_get_trans_result(dev->spi_dev, &ret_trans, portMAX_DELAY);
    if (ret == ESP_OK) {
        memcpy(buf, dev->dma_rx_buf + 1, len);
    }
    return (ret == ESP_OK) ? ICM42688_OK : ICM42688_ERR_CONFIG;
}

/* ============================================================
 *  量程 → 灵敏度
 * ============================================================ */
float icm42688_accel_fs_to_sensitivity(icm42688_accel_fs_t fs)
{
    switch (fs) {
        case ICM42688_ACCEL_16G: return 2048.0f;   /* LSB/g */
        case ICM42688_ACCEL_8G:  return 4096.0f;
        case ICM42688_ACCEL_4G:  return 8192.0f;
        case ICM42688_ACCEL_2G:  return 16384.0f;
        default:                 return 4096.0f;
    }
}

float icm42688_gyro_fs_to_sensitivity(icm42688_gyro_fs_t fs)
{
    switch (fs) {
        case ICM42688_GYRO_2000DPS:  return 16.4f;
        case ICM42688_GYRO_1000DPS:  return 32.8f;
        case ICM42688_GYRO_500DPS:   return 65.5f;
        case ICM42688_GYRO_250DPS:   return 131.0f;
        case ICM42688_GYRO_125DPS:   return 262.0f;
        case ICM42688_GYRO_62_5DPS:  return 524.3f;
        case ICM42688_GYRO_31_25DPS: return 1048.6f;
        case ICM42688_GYRO_15_625DPS:return 2097.2f;
        default:                     return 32.8f;
    }
}

/* ============================================================
 *  API — 寄存器读写
 * ============================================================ */
icm42688_err_t icm42688_read_reg(icm42688_dev_t *dev,
                                  uint8_t reg, uint8_t *val)
{
    if (!dev || !dev->initialized) return ICM42688_ERR_NOT_INIT;
    return spi_read_reg(dev, reg, val);
}

icm42688_err_t icm42688_write_reg(icm42688_dev_t *dev,
                                   uint8_t reg, uint8_t val)
{
    if (!dev || !dev->initialized) return ICM42688_ERR_NOT_INIT;
    return spi_write_reg(dev, reg, val);
}

icm42688_err_t icm42688_read_regs(icm42688_dev_t *dev,
                                   uint8_t reg, uint8_t *buf, size_t len)
{
    if (!dev || !dev->initialized) return ICM42688_ERR_NOT_INIT;
    return spi_read_burst(dev, reg, buf, len);
}

/* 异步提交: 仅启动 DMA, 不等结果 */
icm42688_err_t icm42688_queue_read(icm42688_dev_t *dev)
{
    if (!dev || !dev->initialized) return ICM42688_ERR_NOT_INIT;
    return spi_queue_burst(dev, ICM42688_REG_TEMP_DATA1, 14);
}

/* 收割: 等待 DMA 完成, 解析数据 */
icm42688_err_t icm42688_wait_read(icm42688_dev_t *dev,
                                    icm42688_reading_t *reading)
{
    if (!dev || !dev->initialized || !reading) return ICM42688_ERR_NOT_INIT;
    uint8_t buf[14];
    icm42688_err_t err = spi_collect_burst(dev, buf, sizeof(buf));
    if (err != ICM42688_OK) return err;
    /* 解析数据 */
    int16_t raw_temp = (int16_t)((buf[0] << 8) | buf[1]);
    reading->temp_raw = (float)raw_temp * 0.00253f + 25.0f;
    reading->accel_raw.x = (int16_t)((buf[2]  << 8) | buf[3]);
    reading->accel_raw.y = (int16_t)((buf[4]  << 8) | buf[5]);
    reading->accel_raw.z = (int16_t)((buf[6]  << 8) | buf[7]);
    float accel_sens = icm42688_accel_fs_to_sensitivity(dev->cfg.accel_fs);
    reading->accel_g.x = reading->accel_raw.x / accel_sens - dev->accel_bias.x;
    reading->accel_g.y = reading->accel_raw.y / accel_sens - dev->accel_bias.y;
    reading->accel_g.z = reading->accel_raw.z / accel_sens - dev->accel_bias.z;
    reading->gyro_raw.x = (int16_t)((buf[8]  << 8) | buf[9]);
    reading->gyro_raw.y = (int16_t)((buf[10] << 8) | buf[11]);
    reading->gyro_raw.z = (int16_t)((buf[12] << 8) | buf[13]);
    float gyro_sens = icm42688_gyro_fs_to_sensitivity(dev->cfg.gyro_fs);
    reading->gyro_dps.x = reading->gyro_raw.x / gyro_sens - dev->gyro_bias.x;
    reading->gyro_dps.y = reading->gyro_raw.y / gyro_sens - dev->gyro_bias.y;
    reading->gyro_dps.z = reading->gyro_raw.z / gyro_sens - dev->gyro_bias.z;
    reading->timestamp_us = esp_timer_get_time();
    return ICM42688_OK;
}

/* ============================================================
 *  Bank 切换 (保留供高级配置使用)
 * ============================================================ */
__attribute__((unused))
static icm42688_err_t select_bank(icm42688_dev_t *dev, uint8_t bank)
{
    return spi_write_reg(dev, ICM42688_REG_REG_BANK_SEL, bank);
}

/* ============================================================
 *  API — 初始化
 * ============================================================ */
icm42688_err_t icm42688_init(icm42688_dev_t *dev,
                             const icm42688_spi_cfg_t *spi_cfg,
                             const icm42688_cfg_t *sensor_cfg)
{
    if (!dev || !spi_cfg) return ICM42688_ERR_CONFIG;

    /* 【修改1】将 err 的声明提到函数最前面，确保全函数可用 */
    icm42688_err_t err = ICM42688_OK;

    memset(dev, 0, sizeof(*dev));

    /* ---- 预分配 DMA 缓冲 (生命周期 = dev) ---- */
    dev->dma_tx_buf = (uint8_t *)heap_caps_malloc(DMA_BUF_SIZE, MALLOC_CAP_DMA | MALLOC_CAP_8BIT);
    dev->dma_rx_buf = (uint8_t *)heap_caps_malloc(DMA_BUF_SIZE, MALLOC_CAP_DMA | MALLOC_CAP_8BIT);
    if (!dev->dma_tx_buf || !dev->dma_rx_buf) {
        ESP_LOGE(TAG, "DMA buffer alloc failed");
        /* 【修改2】给 err 赋对应的错误码，并统一进入 cleanup */
        err = ICM42688_ERR_SPI_INIT;
        goto dma_cleanup;
    }

    /* ---- 默认传感器配置 ---- */
    icm42688_cfg_t def_cfg = {
        .accel_fs    = ICM42688_ACCEL_4G,
        .gyro_fs     = ICM42688_GYRO_2000DPS,
        .accel_odr   = ICM42688_ODR_1000HZ,
        .gyro_odr    = ICM42688_ODR_1000HZ,
        .enable_fifo = false,
    };
    if (sensor_cfg) {
        def_cfg = *sensor_cfg;
    }
    dev->cfg = def_cfg;

    /* ---- SPI 总线初始化 (仅首次, 共享总线场景) ---- */
    int host_idx = spi_cfg->spi_host;
    esp_err_t ret;
    if (host_idx >= 0 && host_idx < SPI_HOST_MAX && !s_spi_bus_inited[host_idx]) {
        spi_bus_config_t bus_cfg = {
            .mosi_io_num   = spi_cfg->pin_mosi,
            .miso_io_num   = spi_cfg->pin_miso,
            .sclk_io_num   = spi_cfg->pin_sclk,
            .quadwp_io_num = -1,
            .quadhd_io_num = -1,
            .max_transfer_sz = 64,
        };
        ret = spi_bus_initialize(spi_cfg->spi_host, &bus_cfg, SPI_DMA_CH_AUTO);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "SPI bus init failed: %s", esp_err_to_name(ret));
            /* 【修改3】赋值后 goto */
            err = ICM42688_ERR_SPI_INIT;
            goto dma_cleanup;
        }
        s_spi_bus_inited[host_idx] = true;
        ESP_LOGI(TAG, "SPI%d bus initialized (shared)", spi_cfg->spi_host);
    }

    spi_device_interface_config_t dev_cfg = {
        .clock_speed_hz = spi_cfg->clock_speed_hz ? spi_cfg->clock_speed_hz : 10000000,
        .mode           = 0,  /* CPOL=0, CPHA=0 */
        .spics_io_num   = spi_cfg->pin_cs,
        .queue_size     = 7,
    };
    ret = spi_bus_add_device(spi_cfg->spi_host, &dev_cfg, &dev->spi_dev);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SPI add device failed: %s", esp_err_to_name(ret));
        /* 【修改4】赋值后 goto */
        err = ICM42688_ERR_SPI_INIT;
        goto dma_cleanup;
    }

    dev->initialized = true;
    dev->int_gpio = -1;
    dev->int_sem = NULL;
    dev->int_count = 0;

    /* ---- 软复位 ---- */
    /* 【修改5】直接使用最上方声明的 err */
    err = icm42688_reset(dev);
    if (err != ICM42688_OK) goto dma_cleanup;

    /* ---- 验证 WHO_AM_I ---- */
    uint8_t who = 0;
    err = icm42688_read_reg(dev, ICM42688_REG_WHO_AM_I, &who);
    if (err != ICM42688_OK) goto dma_cleanup;
    if (who != ICM42688_WHO_AM_I_VALUE) {
        ESP_LOGE(TAG, "WHO_AM_I mismatch: 0x%02X (expected 0x%02X)", who, ICM42688_WHO_AM_I_VALUE);
        err = ICM42688_ERR_WHO_AM_I;
        goto dma_cleanup;
    }
    ESP_LOGI(TAG, "WHO_AM_I = 0x%02X ✓", who);

    /* ---- 配置传感器 ---- */
    /* PWR_MGMT0: 使能 Accel + Gyro (low-noise mode), 保留温度 */
    uint8_t pwr = (ICM42688_MODE_LOWNOISE << 2)  /* Accel = low-noise */
                | (ICM42688_MODE_LOWNOISE << 0);  /* Gyro  = low-noise */
    err = spi_write_reg(dev, ICM42688_REG_PWR_MGMT0, pwr);
    if (err != ICM42688_OK) goto dma_cleanup;

    vTaskDelay(pdMS_TO_TICKS(10));  /* 等待电源稳定 */

    /* 陀螺仪: 量程 + ODR */
    uint8_t gyro_cfg = (dev->cfg.gyro_fs & 0x1C) | (dev->cfg.gyro_odr << 3);
    err = spi_write_reg(dev, ICM42688_REG_GYRO_CONFIG0, gyro_cfg);
    if (err != ICM42688_OK) goto dma_cleanup;

    /* 加速度计: 量程 + ODR */
    uint8_t accel_cfg = (dev->cfg.accel_fs & 0x1C) | (dev->cfg.accel_odr << 3);
    err = spi_write_reg(dev, ICM42688_REG_ACCEL_CONFIG0, accel_cfg);
    if (err != ICM42688_OK) goto dma_cleanup;

    /* INT1 配置:  data-ready interrupt, push-pull, active-high */
    err = spi_write_reg(dev, ICM42688_REG_INT_CONFIG,
                        ICM42688_BIT_INT1_POLARITY | ICM42688_BIT_INT1_PUSH_PULL);
    if (err != ICM42688_OK) goto dma_cleanup;

    /* INT_SOURCE0: data-ready → INT1 */
    err = spi_write_reg(dev, ICM42688_REG_INT_SOURCE0, 0x08);  /* UI_DRDY_INT1_EN */
    if (err != ICM42688_OK) goto dma_cleanup;

    ESP_LOGI(TAG, "ICM-42688-P initialized: Accel ±%s, Gyro ±%s, ODR %dHz",
             (dev->cfg.accel_fs == ICM42688_ACCEL_16G) ? "16g" :
             (dev->cfg.accel_fs == ICM42688_ACCEL_8G)  ? "8g"  :
             (dev->cfg.accel_fs == ICM42688_ACCEL_4G)  ? "4g"  : "2g",
             (dev->cfg.gyro_fs == ICM42688_GYRO_2000DPS) ? "2000dps" :
             (dev->cfg.gyro_fs == ICM42688_GYRO_1000DPS) ? "1000dps":
             (dev->cfg.gyro_fs == ICM42688_GYRO_500DPS)  ? "500dps" :
             (dev->cfg.gyro_fs == ICM42688_GYRO_250DPS)  ? "250dps" :
             (dev->cfg.gyro_fs == ICM42688_GYRO_125DPS)  ? "125dps": "?",
             1000);

    /* 如果顺利执行到这里，直接返回 OK */
    return ICM42688_OK;

dma_cleanup:
    /* 【修改6】增加判空，防止分配失败时 free(NULL) 导致不必要的问题 */
    if (dev->dma_tx_buf) heap_caps_free(dev->dma_tx_buf);
    if (dev->dma_rx_buf) heap_caps_free(dev->dma_rx_buf);
    dev->dma_tx_buf = NULL;
    dev->dma_rx_buf = NULL;
    return err;

    /* 【修改7】删除了最底部无法到达的 return ICM42688_OK; */
}

/* ============================================================
 *  API — 软复位
 * ============================================================ */
icm42688_err_t icm42688_reset(icm42688_dev_t *dev)
{
    if (!dev || !dev->initialized) return ICM42688_ERR_NOT_INIT;

    icm42688_err_t err = spi_write_reg(dev, ICM42688_REG_DEVICE_CONFIG,
                                        ICM42688_BIT_SOFT_RESET);
    if (err != ICM42688_OK) return err;

    vTaskDelay(pdMS_TO_TICKS(50));  /* 复位需要 ~30ms */

    /* 复位后寄存器回到 Bank 0 */
    return ICM42688_OK;
}

/* ============================================================
 *  API — 关闭
 * ============================================================ */
void icm42688_deinit(icm42688_dev_t *dev)
{
    if (!dev || !dev->initialized) return;
    /* 进入 standby */
    spi_write_reg(dev, ICM42688_REG_PWR_MGMT0, 0x00);
    spi_bus_remove_device(dev->spi_dev);
    /* 释放 DMA 缓冲 */
    heap_caps_free(dev->dma_tx_buf);
    heap_caps_free(dev->dma_rx_buf);
    dev->dma_tx_buf = NULL;
    dev->dma_rx_buf = NULL;
    dev->initialized = false;
}

/* ============================================================
 *  API — 读取 (polling, burst read)
 * ============================================================ */

/**
 * ICM42688 数据寄存器布局 (Bank 0):
 *   0x1D: TEMP_DATA1  (bit[11:4])
 *   0x1E: TEMP_DATA0  (bit[3:0])
 *   0x1F-0x24: ACCEL_X, Y, Z (each 2 bytes, big-endian)
 *   0x25-0x2A: GYRO_X, Y, Z  (each 2 bytes, big-endian)
 *   共 14 bytes: 温度 2B + 加速度 6B + 陀螺仪 6B
 */
icm42688_err_t icm42688_read_polling(icm42688_dev_t *dev,
                                     icm42688_reading_t *reading)
{
    if (!dev || !dev->initialized) return ICM42688_ERR_NOT_INIT;
    if (!reading) return ICM42688_ERR_CONFIG;

    uint8_t buf[14];
    icm42688_err_t err = spi_read_burst(dev, ICM42688_REG_TEMP_DATA1, buf, sizeof(buf));
    if (err != ICM42688_OK) return err;

    /* --- 温度 (16-bit signed, 灵敏度 ~0.00253°C/LSB, offset -15.0°C) --- */
    int16_t raw_temp = (int16_t)((buf[0] << 8) | buf[1]);
    reading->temp_raw = (float)raw_temp * 0.00253f + 25.0f;  /* 公式参考 datasheet */

    /* --- 加速度 (big-endian int16) --- */
    reading->accel_raw.x = (int16_t)((buf[2]  << 8) | buf[3]);
    reading->accel_raw.y = (int16_t)((buf[4]  << 8) | buf[5]);
    reading->accel_raw.z = (int16_t)((buf[6]  << 8) | buf[7]);

    float accel_sens = icm42688_accel_fs_to_sensitivity(dev->cfg.accel_fs);
    reading->accel_g.x = reading->accel_raw.x / accel_sens - dev->accel_bias.x;
    reading->accel_g.y = reading->accel_raw.y / accel_sens - dev->accel_bias.y;
    reading->accel_g.z = reading->accel_raw.z / accel_sens - dev->accel_bias.z;

    /* --- 陀螺仪 (big-endian int16) --- */
    reading->gyro_raw.x = (int16_t)((buf[8]  << 8) | buf[9]);
    reading->gyro_raw.y = (int16_t)((buf[10] << 8) | buf[11]);
    reading->gyro_raw.z = (int16_t)((buf[12] << 8) | buf[13]);

    float gyro_sens = icm42688_gyro_fs_to_sensitivity(dev->cfg.gyro_fs);
    reading->gyro_dps.x = reading->gyro_raw.x / gyro_sens - dev->gyro_bias.x;
    reading->gyro_dps.y = reading->gyro_raw.y / gyro_sens - dev->gyro_bias.y;
    reading->gyro_dps.z = reading->gyro_raw.z / gyro_sens - dev->gyro_bias.z;

    /* --- 时间戳 --- */
    reading->timestamp_us = esp_timer_get_time();

    return ICM42688_OK;
}

/* ============================================================
 *  API — 偏移校准
 * ============================================================ */
icm42688_err_t icm42688_calibrate_bias(icm42688_dev_t *dev, uint32_t samples)
{
    if (!dev || !dev->initialized) return ICM42688_ERR_NOT_INIT;
    if (samples == 0) samples = 200;

    ESP_LOGI(TAG, "开始校准, 采集 %lu 次...", samples);

    double ax_sum = 0, ay_sum = 0, az_sum = 0;
    double gx_sum = 0, gy_sum = 0, gz_sum = 0;

    icm42688_reading_t raw;

    for (uint32_t i = 0; i < samples; i++) {
        icm42688_err_t err = icm42688_read_polling(dev, &raw);
        if (err != ICM42688_OK) return err;

        ax_sum += raw.accel_g.x;
        ay_sum += raw.accel_g.y;
        az_sum += raw.accel_g.z;
        gx_sum += raw.gyro_dps.x;
        gy_sum += raw.gyro_dps.y;
        gz_sum += raw.gyro_dps.z;

        vTaskDelay(pdMS_TO_TICKS(1));  /* 等待下一次数据就绪 (1kHz → 1ms) */
    }

    dev->accel_bias.x = (float)(ax_sum / samples);
    dev->accel_bias.y = (float)(ay_sum / samples);
    /* 加速度计 Z 轴: 去掉 1g 重力 */
    dev->accel_bias.z = (float)(az_sum / samples) - 1.0f;

    dev->gyro_bias.x = (float)(gx_sum / samples);
    dev->gyro_bias.y = (float)(gy_sum / samples);
    dev->gyro_bias.z = (float)(gz_sum / samples);

    ESP_LOGI(TAG, "校准完成:");
    ESP_LOGI(TAG, "  Accel bias: [%.4f, %.4f, %.4f] g",
             dev->accel_bias.x, dev->accel_bias.y, dev->accel_bias.z);
    ESP_LOGI(TAG, "  Gyro  bias: [%.4f, %.4f, %.4f] dps",
             dev->gyro_bias.x, dev->gyro_bias.y, dev->gyro_bias.z);

    return ICM42688_OK;
}

icm42688_axis3f_t icm42688_get_accel_bias(const icm42688_dev_t *dev)
{
    return dev->accel_bias;
}

icm42688_axis3f_t icm42688_get_gyro_bias(const icm42688_dev_t *dev)
{
    return dev->gyro_bias;
}

void icm42688_set_accel_bias(icm42688_dev_t *dev, icm42688_axis3f_t bias)
{
    dev->accel_bias = bias;
}

void icm42688_set_gyro_bias(icm42688_dev_t *dev, icm42688_axis3f_t bias)
{
    dev->gyro_bias = bias;
}

/* ============================================================
 *  API — 中断驱动读取
 * ============================================================ */

/*
 * ISR: 设置事件组位 + 释放信号量 (双路同步支持)
 */
static void IRAM_ATTR icm42688_int_isr(void *arg)
{
    icm42688_dev_t *dev = (icm42688_dev_t *)arg;
    dev->int_count++;
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    /* 信号量: 供单路 wait_drdy 使用 */
    if (dev->int_sem) {
        xSemaphoreGiveFromISR(dev->int_sem, &xHigherPriorityTaskWoken);
    }
    /* 事件组: 供双路同步 wait_drdy_group 使用 */
    if (dev->int_evtgrp) {
        /* 用 int_gpio 的低位作为事件位 (0=IMU-A, 1=IMU-B) */
        int bit = (dev->int_gpio == 0) ? 0 : 1;
        xEventGroupSetBitsFromISR(dev->int_evtgrp, (1 << bit), &xHigherPriorityTaskWoken);
    }

    if (xHigherPriorityTaskWoken) {
        portYIELD_FROM_ISR();
    }
}

icm42688_err_t icm42688_init_interrupt(icm42688_dev_t *dev, int int_pin)
{
    if (!dev || !dev->initialized || int_pin < 0) return ICM42688_ERR_CONFIG;

    /* 创建二值信号量 */
    dev->int_sem = xSemaphoreCreateBinary();
    if (!dev->int_sem) {
        ESP_LOGE(TAG, "Failed to create interrupt semaphore");
        return ICM42688_ERR_CONFIG;
    }

    /* 创建事件组 (用于双路同步) */
    dev->int_evtgrp = xEventGroupCreate();
    if (!dev->int_evtgrp) {
        ESP_LOGE(TAG, "Failed to create interrupt event group");
        vSemaphoreDelete(dev->int_sem);
        dev->int_sem = NULL;
        return ICM42688_ERR_CONFIG;
    }

    /* 配置 GPIO 为输入, 下降沿触发 */
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << int_pin),
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_NEGEDGE,
    };
    esp_err_t ret = gpio_config(&io_conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "GPIO config failed for INT pin %d: %s", int_pin, esp_err_to_name(ret));
        vSemaphoreDelete(dev->int_sem);
        dev->int_sem = NULL;
        return ICM42688_ERR_CONFIG;
    }

    /* 安装 GPIO ISR 服务 (全局只需一次, 重复调用安全) */
    gpio_install_isr_service(0);

    /* 绑定 ISR 到此 GPIO */
    ret = gpio_isr_handler_add(int_pin, icm42688_int_isr, (void *)dev);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "ISR add failed for pin %d: %s", int_pin, esp_err_to_name(ret));
        vSemaphoreDelete(dev->int_sem);
        dev->int_sem = NULL;
        return ICM42688_ERR_CONFIG;
    }

    dev->int_gpio = int_pin;
    ESP_LOGI(TAG, "Interrupt configured on GPIO %d (falling edge, DRDY)", int_pin);

    return ICM42688_OK;
}

icm42688_err_t icm42688_wait_drdy(icm42688_dev_t *dev, uint32_t timeout_ms)
{
    if (!dev || !dev->int_sem) return ICM42688_ERR_NOT_INIT;

    TickType_t ticks = (timeout_ms == 0) ? 1 : pdMS_TO_TICKS(timeout_ms);
    if (timeout_ms == UINT32_MAX) {
        ticks = portMAX_DELAY;
    }

    if (xSemaphoreTake(dev->int_sem, ticks) == pdTRUE) {
        return ICM42688_OK;
    }
    return ICM42688_ERR_TIMEOUT;
}

uint32_t icm42688_get_int_count(const icm42688_dev_t *dev)
{
    return dev ? dev->int_count : 0;
}

/* ============================================================
 *  双路 EventGroup 同步等待
 *  同时等待 dev_a 和 dev_b 的中断事件位,
 *  任一路触发即返回, 实现真正的并发等待。
 * ============================================================ */
icm42688_err_t icm42688_wait_drdy_group(icm42688_dev_t *dev_a,
                                          icm42688_dev_t *dev_b,
                                          uint32_t timeout_ms)
{
    if (!dev_a || !dev_b) return ICM42688_ERR_CONFIG;
    if (!dev_a->int_evtgrp || !dev_b->int_evtgrp) return ICM42688_ERR_NOT_INIT;

    /* 两个事件组各用 bit0, 合并为同一组后同时等待 bit0 | bit1 */
    EventBits_t bits_a = (1 << 0);  /* IMU-A 的事件位 */
    EventBits_t bits_b = (1 << 1);  /* IMU-B 的事件位 */
    EventBits_t wait_bits = bits_a | bits_b;

    /* 使用 IMU-A 的事件组作为共享组 (两路 ISR 都往同一组写) */
    EventGroupHandle_t shared = dev_a->int_evtgrp;

    /* 清除之前的位, 避免读到旧数据 */
    xEventGroupClearBits(shared, wait_bits);

    TickType_t ticks = pdMS_TO_TICKS(timeout_ms);
    if (timeout_ms == 0 || timeout_ms == UINT32_MAX) ticks = portMAX_DELAY;

    /* 等待: 任一路触发即返回 (等待 OR 条件) */
    EventBits_t triggered = xEventGroupWaitBits(shared, wait_bits,
                                                 pdFALSE,  /* 不清除位 */
                                                 pdFALSE,  /* OR: 任一位即可 */
                                                 ticks);

    if (triggered == 0) {
        return ICM42688_ERR_TIMEOUT;
    }
    return ICM42688_OK;
}
