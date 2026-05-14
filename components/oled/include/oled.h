/**
 * @file oled.h
 * @brief 0.49寸 OLED (SSD1306, 64x32, I2C) 驱动
 */

#ifndef OLED_H
#define OLED_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================
 *  引脚定义
 * ============================================================ */
#define OLED_PIN_PWR_EN    35    /* 电源使能, 高电平使能 (避开 PSRAM GPIO 33~37) */
#define OLED_PIN_RES       36    /* 硬件复位, 低电平有效 */
#define OLED_PIN_SCL       37    /* I2C 时钟 */
#define OLED_PIN_SDA       38    /* I2C 数据 */
#define OLED_I2C_ADDR      0x3C /* 7-bit I2C 地址 */
#define OLED_I2C_PORT       0   /* I2C_NUM_0 */
#define OLED_I2C_FREQ   400000  /* 400KHz 快速模式 */

/* ============================================================
 *  屏幕参数
 * ============================================================ */
#define OLED_WIDTH         64
#define OLED_HEIGHT        32
#define OLED_BUF_SIZE     (OLED_WIDTH * OLED_HEIGHT / 8)  /* 256 字节 */

/* ============================================================
 *  返回码
 * ============================================================ */
typedef enum {
    OLED_OK = 0,
    OLED_ERR_INIT,
    OLED_ERR_I2C,
} oled_err_t;

/* ============================================================
 *  API
 * ============================================================ */

/**
 * @brief 初始化屏幕 (PWR_EN 使能 → 延时 → RES 复位序列 → I2C → SSD1306 命令)
 */
oled_err_t oled_init(void);

/**
 * @brief 清空显存 Buffer (全零)
 */
void oled_clear(void);

/**
 * @brief 将 Buffer 刷新到 OLED RAM
 */
void oled_refresh(void);

/**
 * @brief 显示字符串 (6x8 字体)
 * @param x  列起始 (0~57)
 * @param y  行起始 (0~3, 每行 8 像素)
 * @param str 字符串
 */
void oled_show_string(uint8_t x, uint8_t y, const char *str);

/**
 * @brief 在指定像素位置绘制单个像素
 */
void oled_set_pixel(uint8_t x, uint8_t y, bool on);

/**
 * @brief 清除指定矩形区域
 */
void oled_clear_rect(uint8_t x, uint8_t y, uint8_t w, uint8_t h);

#ifdef __cplusplus
}
#endif

#endif /* OLED_H */
