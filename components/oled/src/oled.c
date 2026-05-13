/**
 * @file oled.c
 * @brief SSD1306 I2C 驱动 (64x32, ESP-IDF 5.x 新版 I2C Master API)
 *
 * 迁移说明: 从弃用的 driver/i2c.h 迁移至 driver/i2c_master.h
 * 新架构: Bus (总线) + Device (设备) 两级模型, 线程安全, 多设备共享
 */

#include "oled.h"
#include "driver/i2c_master.h"  /* [迁移] IDF 5.x 新版 I2C Master 驱动 */
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include <string.h>

static const char *TAG = "oled";

/* 显存 Buffer: 64x32 / 8 = 256 字节 (Horizontal addressing) */
static uint8_t s_buf[OLED_BUF_SIZE];

/* [迁移] 新版 I2C 句柄: Bus + Device 两级架构 */
static i2c_master_bus_handle_t s_i2c_bus_handle = NULL;
static i2c_master_dev_handle_t s_oled_dev_handle = NULL;

/* ============================================================
 *  I2C 底层写入 (IDF 5.x 新 API)
 * ============================================================ */
static esp_err_t oled_write_cmd(uint8_t cmd)
{
    uint8_t buf[2] = { 0x00, cmd };  /* Co=0, D/C#=0 */
    return i2c_master_transmit(s_oled_dev_handle, buf, 2, 100);
}

__attribute__((unused))
static esp_err_t oled_write_data(uint8_t data)
{
    uint8_t buf[2] = { 0x40, data };  /* Co=0, D/C#=1 */
    return i2c_master_transmit(s_oled_dev_handle, buf, 2, 100);
}

static esp_err_t oled_write_data_bulk(const uint8_t *data, size_t len)
{
    /* 带前缀 0x40 的批量写入 */
    uint8_t *tmp = malloc(len + 1);
    if (!tmp) return ESP_ERR_NO_MEM;
    tmp[0] = 0x40;
    memcpy(tmp + 1, data, len);
    esp_err_t ret = i2c_master_transmit(s_oled_dev_handle, tmp, len + 1, 500);
    free(tmp);
    return ret;
}

/* ============================================================
 *  I2C 初始化 (IDF 5.x Bus+Device 架构)
 * ============================================================ */
static esp_err_t i2c_master_init(void)
{
    /* 1. 创建 I2C 主机总线 */
    i2c_master_bus_config_t bus_config = {
        .i2c_port   = OLED_I2C_PORT,
        .sda_io_num = OLED_PIN_SDA,
        .scl_io_num = OLED_PIN_SCL,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .intr_priority     = 0,
        .flags.enable_internal_pullup = true,
    };

    esp_err_t ret = i2c_new_master_bus(&bus_config, &s_i2c_bus_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C bus creation failed: %s", esp_err_to_name(ret));
        return ret;
    }

    /* 2. 向总线添加 OLED 设备 (自带地址 + 速率) */
    i2c_device_config_t dev_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address  = OLED_I2C_ADDR,
        .scl_speed_hz    = OLED_I2C_FREQ,
    };

    ret = i2c_master_bus_add_device(s_i2c_bus_handle, &dev_config, &s_oled_dev_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C add device failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "I2C master v5.x init: SDA=%d SCL=%d @ %dHz",
             OLED_PIN_SDA, OLED_PIN_SCL, OLED_I2C_FREQ);
    return ESP_OK;
}

/* ============================================================
 *  SSD1306 64x32 初始化命令序列
 * ============================================================ */
static void ssd1306_init_cmds(void)
{
    oled_write_cmd(0xAE);        /* Display OFF */
    oled_write_cmd(0xD5);
    oled_write_cmd(0x80);        /* Clock div */
    oled_write_cmd(0xA8);
    oled_write_cmd(0x1F);        /* Multiplex Ratio = 31 (适配 32 行) */
    oled_write_cmd(0xD3);
    oled_write_cmd(0x00);        /* Display Offset = 0 */
    oled_write_cmd(0x40);        /* Start Line = 0 */
    oled_write_cmd(0x8D);
    oled_write_cmd(0x14);        /* Charge Pump ON */
    oled_write_cmd(0x20);
    oled_write_cmd(0x00);        /* Horizontal addressing */
    oled_write_cmd(0xA1);        /* Segment Re-map (左右镜像) */
    oled_write_cmd(0xC8);        /* COM Scan Direction (上下镜像) */
    oled_write_cmd(0xDA);
    oled_write_cmd(0x12);        /* COM Pins (64x32 专用) */
    oled_write_cmd(0x81);
    oled_write_cmd(0x7F);        /* Contrast = 127 */
    oled_write_cmd(0xD9);
    oled_write_cmd(0x1F);        /* Pre-charge */
    oled_write_cmd(0xDB);
    oled_write_cmd(0x40);        /* VCOMH */
    oled_write_cmd(0xA4);        /* Display from RAM */
    oled_write_cmd(0xA6);        /* Normal display */
    oled_write_cmd(0xAF);        /* Display ON */
}

/* ============================================================
 *  API — 初始化
 * ============================================================ */
oled_err_t oled_init(void)
{
    ESP_LOGI(TAG, "OLED 初始化...");

    /* ============================================================
     * [致命冲突检测] ESP32-S3-WROOM-1-N16R8 的 Octal PSRAM 占用
     * GPIO 33~37。任何 gpio_config / i2c_driver_install 都会断开
     * MSPI 控制器对 PSRAM 引脚的所有权，导致后续 cache 崩溃
     * (TG1WDT_SYS_RST at panic_enable_cache)。
     * 如果 OLED 引脚落在 33~37 范围内，必须立即终止初始化。
     * ============================================================ */
    #define PSRAM_GPIO_MIN  33
    #define PSRAM_GPIO_MAX  37
    if ((OLED_PIN_PWR_EN >= PSRAM_GPIO_MIN && OLED_PIN_PWR_EN <= PSRAM_GPIO_MAX) ||
        (OLED_PIN_RES    >= PSRAM_GPIO_MIN && OLED_PIN_RES    <= PSRAM_GPIO_MAX) ||
        (OLED_PIN_SCL    >= PSRAM_GPIO_MIN && OLED_PIN_SCL    <= PSRAM_GPIO_MAX) ||
        (OLED_PIN_SDA    >= PSRAM_GPIO_MIN && OLED_PIN_SDA    <= PSRAM_GPIO_MAX)) {
        ESP_LOGE(TAG, "===========================================================");
        ESP_LOGE(TAG, "OLED 引脚与 Octal PSRAM 冲突!");
        ESP_LOGE(TAG, "  PWR_EN=IO%d  RES=IO%d  SCL=IO%d  SDA=IO%d",
                 OLED_PIN_PWR_EN, OLED_PIN_RES, OLED_PIN_SCL, OLED_PIN_SDA);
        ESP_LOGE(TAG, "  PSRAM 占用 GPIO %d~%d, 不能用于 OLED!",
                 PSRAM_GPIO_MIN, PSRAM_GPIO_MAX);
        ESP_LOGE(TAG, "  请将 OLED 移至 GPIO 0~32 区域 (避开 Flash GPIO 26~32)");
        ESP_LOGE(TAG, "===========================================================");
        return OLED_ERR_I2C;  /* 不碰任何 GPIO, 从根源保护 PSRAM */
    }

    /* 1. 电源使能 */
    gpio_config_t pwr_io = {
        .pin_bit_mask = (1ULL << OLED_PIN_PWR_EN),
        .mode         = GPIO_MODE_OUTPUT,
    };
    gpio_config(&pwr_io);
    gpio_set_level(OLED_PIN_PWR_EN, 1);
    ESP_LOGI(TAG, "PWR_EN(IO%d) = HIGH", OLED_PIN_PWR_EN);

    /* 2. 等待 LDO 稳压 */
    vTaskDelay(pdMS_TO_TICKS(50));

    /* [F7] 3. 物理阻抗探测 (Bus Sanity Check)
     * 在 i2c_driver_install 之前, 用 GPIO 内部弱下拉 + 外部上拉的
     * 阻抗分压原理嗅探 SDA/SCL 电平, 判定总线上是否存在屏幕。
     * 无屏时 SDA/SCL 悬空 → 内部下拉拉低 → 检测到低电平 → 安全退出。 */
    ESP_LOGI(TAG, "执行 I2C 总线物理阻抗探测...");
    gpio_config_t probe_io = {
        .pin_bit_mask = (1ULL << OLED_PIN_SDA) | (1ULL << OLED_PIN_SCL),
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
    };
    gpio_config(&probe_io);
    vTaskDelay(pdMS_TO_TICKS(5));  /* 等待内部下拉电平稳定 */

    int sda_level = gpio_get_level(OLED_PIN_SDA);
    int scl_level = gpio_get_level(OLED_PIN_SCL);

    /* 恢复引脚默认状态, 交还给 I2C 控制器 */
    gpio_reset_pin(OLED_PIN_SDA);
    gpio_reset_pin(OLED_PIN_SCL);

    if (sda_level == 0 || scl_level == 0) {
        ESP_LOGW(TAG, "探测失败: SDA=%d SCL=%d (被下拉为低, 无外部上拉电阻, 无屏幕)",
                 sda_level, scl_level);
        return OLED_ERR_I2C;  /* 不碰 i2c_master_init, 从根源避免硬件死锁 */
    }

    ESP_LOGI(TAG, "探测成功: SDA=%d SCL=%d (检测到外部上拉电阻, 屏幕就绪)",
             sda_level, scl_level);

    /* 4. 硬件复位时序 (RES#) */
    gpio_config_t res_io = {
        .pin_bit_mask = (1ULL << OLED_PIN_RES),
        .mode         = GPIO_MODE_OUTPUT,
    };
    gpio_config(&res_io);

    gpio_set_level(OLED_PIN_RES, 1);  /* RES# = HIGH (正常) */
    vTaskDelay(pdMS_TO_TICKS(10));

    gpio_set_level(OLED_PIN_RES, 0);  /* RES# = LOW  (复位) */
    vTaskDelay(pdMS_TO_TICKS(20));

    gpio_set_level(OLED_PIN_RES, 1);  /* RES# = HIGH (复位结束) */
    vTaskDelay(pdMS_TO_TICKS(20));
    ESP_LOGI(TAG, "RES# 复位序列完成");

    /* 5. I2C 初始化 */
    esp_err_t ret = i2c_master_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C init failed");
        return OLED_ERR_I2C;
    }

    /* 6. SSD1306 初始化命令 */
    ssd1306_init_cmds();

    /* 6. 清屏 */
    oled_clear();
    oled_refresh();

    ESP_LOGI(TAG, "OLED 初始化完成 (%dx%d)", OLED_WIDTH, OLED_HEIGHT);
    return OLED_OK;
}

/* ============================================================
 *  API — 清空显存
 * ============================================================ */
void oled_clear(void)
{
    memset(s_buf, 0, OLED_BUF_SIZE);
}

/* ============================================================
 *  API — 刷新到 OLED
 * ============================================================ */
void oled_refresh(void)
{
    /* 设置列地址范围: 0 ~ 63 */
    oled_write_cmd(0x21);
    oled_write_cmd(0);
    oled_write_cmd(OLED_WIDTH - 1);

    /* 设置页地址范围: 0 ~ 3 (64x32 / 8 = 4 页) */
    oled_write_cmd(0x22);
    oled_write_cmd(0);
    oled_write_cmd((OLED_HEIGHT / 8) - 1);

    /* 批量写入显存 */
    oled_write_data_bulk(s_buf, OLED_BUF_SIZE);
}

/* ============================================================
 *  API — 设置像素
 * ============================================================ */
void oled_set_pixel(uint8_t x, uint8_t y, bool on)
{
    if (x >= OLED_WIDTH || y >= OLED_HEIGHT) return;
    uint16_t idx = (y / 8) * OLED_WIDTH + x;
    uint8_t  bit = y % 8;
    if (on) {
        s_buf[idx] |= (1 << bit);
    } else {
        s_buf[idx] &= ~(1 << bit);
    }
}

/* ============================================================
 *  API — 清除矩形区域
 * ============================================================ */
void oled_clear_rect(uint8_t x, uint8_t y, uint8_t w, uint8_t h)
{
    for (uint8_t dy = 0; dy < h; dy++) {
        for (uint8_t dx = 0; dx < w; dx++) {
            oled_set_pixel(x + dx, y + dy, false);
        }
    }
}

/* ============================================================
 *  6x8 ASCII 字体 (可打印字符 0x20~0x7E)
 * ============================================================ */
static const uint8_t font_6x8[][6] = {
    { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 }, /* 0x20 (space) */
    { 0x00, 0x00, 0x5F, 0x00, 0x00, 0x00 }, /* 0x21 ! */
    { 0x00, 0x07, 0x00, 0x07, 0x00, 0x00 }, /* 0x22 " */
    { 0x14, 0x7F, 0x14, 0x7F, 0x14, 0x00 }, /* 0x23 # */
    { 0x24, 0x2A, 0x7F, 0x2A, 0x12, 0x00 }, /* 0x24 $ */
    { 0x23, 0x13, 0x08, 0x64, 0x62, 0x00 }, /* 0x25 % */
    { 0x36, 0x49, 0x55, 0x22, 0x50, 0x00 }, /* 0x26 & */
    { 0x00, 0x05, 0x03, 0x00, 0x00, 0x00 }, /* 0x27 ' */
    { 0x00, 0x1C, 0x22, 0x41, 0x00, 0x00 }, /* 0x28 ( */
    { 0x00, 0x41, 0x22, 0x1C, 0x00, 0x00 }, /* 0x29 ) */
    { 0x14, 0x08, 0x3E, 0x08, 0x14, 0x00 }, /* 0x2A * */
    { 0x08, 0x08, 0x3E, 0x08, 0x08, 0x00 }, /* 0x2B + */
    { 0x00, 0x50, 0x30, 0x00, 0x00, 0x00 }, /* 0x2C , */
    { 0x08, 0x08, 0x08, 0x08, 0x08, 0x00 }, /* 0x2D - */
    { 0x00, 0x60, 0x60, 0x00, 0x00, 0x00 }, /* 0x2E . */
    { 0x20, 0x10, 0x08, 0x04, 0x02, 0x00 }, /* 0x2F / */
    { 0x3E, 0x51, 0x49, 0x45, 0x3E, 0x00 }, /* 0x30 0 */
    { 0x00, 0x42, 0x7F, 0x40, 0x00, 0x00 }, /* 0x31 1 */
    { 0x42, 0x61, 0x51, 0x49, 0x46, 0x00 }, /* 0x32 2 */
    { 0x21, 0x41, 0x45, 0x4B, 0x31, 0x00 }, /* 0x33 3 */
    { 0x18, 0x14, 0x12, 0x7F, 0x10, 0x00 }, /* 0x34 4 */
    { 0x27, 0x45, 0x45, 0x45, 0x39, 0x00 }, /* 0x35 5 */
    { 0x3C, 0x4A, 0x49, 0x49, 0x30, 0x00 }, /* 0x36 6 */
    { 0x01, 0x71, 0x09, 0x05, 0x03, 0x00 }, /* 0x37 7 */
    { 0x36, 0x49, 0x49, 0x49, 0x36, 0x00 }, /* 0x38 8 */
    { 0x46, 0x49, 0x49, 0x29, 0x1E, 0x00 }, /* 0x39 9 */
    { 0x00, 0x36, 0x36, 0x00, 0x00, 0x00 }, /* 0x3A : */
    { 0x00, 0x56, 0x36, 0x00, 0x00, 0x00 }, /* 0x3B ; */
    { 0x08, 0x14, 0x22, 0x41, 0x00, 0x00 }, /* 0x3C < */
    { 0x14, 0x14, 0x14, 0x14, 0x14, 0x00 }, /* 0x3D = */
    { 0x00, 0x41, 0x22, 0x14, 0x08, 0x00 }, /* 0x3E > */
    { 0x02, 0x01, 0x51, 0x09, 0x06, 0x00 }, /* 0x3F ? */
    { 0x32, 0x49, 0x79, 0x41, 0x3E, 0x00 }, /* 0x40 @ */
    { 0x7E, 0x11, 0x11, 0x11, 0x7E, 0x00 }, /* 0x41 A */
    { 0x7F, 0x49, 0x49, 0x49, 0x36, 0x00 }, /* 0x42 B */
    { 0x3E, 0x41, 0x41, 0x41, 0x22, 0x00 }, /* 0x43 C */
    { 0x7F, 0x41, 0x41, 0x22, 0x1C, 0x00 }, /* 0x44 D */
    { 0x7F, 0x49, 0x49, 0x49, 0x41, 0x00 }, /* 0x45 E */
    { 0x7F, 0x09, 0x09, 0x09, 0x01, 0x00 }, /* 0x46 F */
    { 0x3E, 0x41, 0x41, 0x51, 0x32, 0x00 }, /* 0x47 G */
    { 0x7F, 0x08, 0x08, 0x08, 0x7F, 0x00 }, /* 0x48 H */
    { 0x00, 0x41, 0x7F, 0x41, 0x00, 0x00 }, /* 0x49 I */
    { 0x20, 0x40, 0x41, 0x3F, 0x01, 0x00 }, /* 0x4A J */
    { 0x7F, 0x08, 0x14, 0x22, 0x41, 0x00 }, /* 0x4B K */
    { 0x7F, 0x40, 0x40, 0x40, 0x40, 0x00 }, /* 0x4C L */
    { 0x7F, 0x02, 0x0C, 0x02, 0x7F, 0x00 }, /* 0x4D M */
    { 0x7F, 0x04, 0x08, 0x10, 0x7F, 0x00 }, /* 0x4E N */
    { 0x3E, 0x41, 0x41, 0x41, 0x3E, 0x00 }, /* 0x4F O */
    { 0x7F, 0x09, 0x09, 0x09, 0x06, 0x00 }, /* 0x50 P */
    { 0x3E, 0x41, 0x51, 0x21, 0x5E, 0x00 }, /* 0x51 Q */
    { 0x7F, 0x09, 0x19, 0x29, 0x46, 0x00 }, /* 0x52 R */
    { 0x46, 0x49, 0x49, 0x49, 0x31, 0x00 }, /* 0x53 S */
    { 0x01, 0x01, 0x7F, 0x01, 0x01, 0x00 }, /* 0x54 T */
    { 0x3F, 0x40, 0x40, 0x40, 0x3F, 0x00 }, /* 0x55 U */
    { 0x1F, 0x20, 0x40, 0x20, 0x1F, 0x00 }, /* 0x56 V */
    { 0x3F, 0x40, 0x38, 0x40, 0x3F, 0x00 }, /* 0x57 W */
    { 0x63, 0x14, 0x08, 0x14, 0x63, 0x00 }, /* 0x58 X */
    { 0x07, 0x08, 0x70, 0x08, 0x07, 0x00 }, /* 0x59 Y */
    { 0x61, 0x51, 0x49, 0x45, 0x43, 0x00 }, /* 0x5A Z */
    { 0x00, 0x7F, 0x41, 0x41, 0x00, 0x00 }, /* 0x5B [ */
    { 0x02, 0x04, 0x08, 0x10, 0x20, 0x00 }, /* 0x5C backslash */
    { 0x00, 0x41, 0x41, 0x7F, 0x00, 0x00 }, /* 0x5D ] */
    { 0x04, 0x02, 0x01, 0x02, 0x04, 0x00 }, /* 0x5E ^ */
    { 0x40, 0x40, 0x40, 0x40, 0x40, 0x00 }, /* 0x5F _ */
    { 0x00, 0x01, 0x02, 0x04, 0x00, 0x00 }, /* 0x60 ` */
    { 0x20, 0x54, 0x54, 0x54, 0x78, 0x00 }, /* 0x61 a */
    { 0x7F, 0x48, 0x44, 0x44, 0x38, 0x00 }, /* 0x62 b */
    { 0x38, 0x44, 0x44, 0x44, 0x20, 0x00 }, /* 0x63 c */
    { 0x38, 0x44, 0x44, 0x48, 0x7F, 0x00 }, /* 0x64 d */
    { 0x38, 0x54, 0x54, 0x54, 0x18, 0x00 }, /* 0x65 e */
    { 0x08, 0x7E, 0x09, 0x01, 0x02, 0x00 }, /* 0x66 f */
    { 0x0C, 0x52, 0x52, 0x52, 0x3E, 0x00 }, /* 0x67 g */
    { 0x7F, 0x08, 0x04, 0x04, 0x78, 0x00 }, /* 0x68 h */
    { 0x00, 0x44, 0x7D, 0x40, 0x00, 0x00 }, /* 0x69 i */
    { 0x20, 0x40, 0x44, 0x3D, 0x00, 0x00 }, /* 0x6A j */
    { 0x7F, 0x10, 0x28, 0x44, 0x00, 0x00 }, /* 0x6B k */
    { 0x00, 0x41, 0x7F, 0x40, 0x00, 0x00 }, /* 0x6C l */
    { 0x7C, 0x04, 0x18, 0x04, 0x78, 0x00 }, /* 0x6D m */
    { 0x7C, 0x08, 0x04, 0x04, 0x78, 0x00 }, /* 0x6E n */
    { 0x38, 0x44, 0x44, 0x44, 0x38, 0x00 }, /* 0x6F o */
    { 0x7C, 0x14, 0x14, 0x14, 0x08, 0x00 }, /* 0x70 p */
    { 0x08, 0x14, 0x14, 0x18, 0x7C, 0x00 }, /* 0x71 q */
    { 0x7C, 0x08, 0x04, 0x04, 0x08, 0x00 }, /* 0x72 r */
    { 0x48, 0x54, 0x54, 0x54, 0x20, 0x00 }, /* 0x73 s */
    { 0x04, 0x3F, 0x44, 0x40, 0x20, 0x00 }, /* 0x74 t */
    { 0x3C, 0x40, 0x40, 0x20, 0x7C, 0x00 }, /* 0x75 u */
    { 0x1C, 0x20, 0x40, 0x20, 0x1C, 0x00 }, /* 0x76 v */
    { 0x3C, 0x40, 0x30, 0x40, 0x3C, 0x00 }, /* 0x77 w */
    { 0x44, 0x28, 0x10, 0x28, 0x44, 0x00 }, /* 0x78 x */
    { 0x0C, 0x50, 0x50, 0x50, 0x3C, 0x00 }, /* 0x79 y */
    { 0x44, 0x64, 0x54, 0x4C, 0x44, 0x00 }, /* 0x7A z */
    { 0x00, 0x08, 0x36, 0x41, 0x00, 0x00 }, /* 0x7B { */
    { 0x00, 0x00, 0x7F, 0x00, 0x00, 0x00 }, /* 0x7C | */
    { 0x00, 0x41, 0x36, 0x08, 0x00, 0x00 }, /* 0x7D } */
    { 0x10, 0x08, 0x08, 0x10, 0x08, 0x00 }, /* 0x7E ~ */
};

/* ============================================================
 *  API — 显示字符串
 * ============================================================ */
void oled_show_string(uint8_t x, uint8_t y, const char *str)
{
    if (!str) return;
    while (*str) {
        uint8_t c = *str;
        if (c < 0x20 || c > 0x7E) c = 0x20;  /* 不可打印字符 → 空格 */

        /* 字体数据是按列存储的 (每字节 1 列, 8 像素高) */
        for (int col = 0; col < 6; col++) {
            uint8_t line = font_6x8[c - 0x20][col];
            for (int row = 0; row < 8; row++) {
                bool pixel = (line >> row) & 1;
                oled_set_pixel(x + col, y + row, pixel);
            }
        }

        x += 6;
        if (x > OLED_WIDTH - 6) break;  /* 换行或停止 */
        str++;
    }
}
