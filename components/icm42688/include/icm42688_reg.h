/**
 * @file icm42688_reg.h
 * @brief ICM-42688-P 寄存器定义
 *
 * 资料来源: TDK ICM-42688-P Product Specification (DS-000292)
 */

#ifndef ICM42688_REG_H
#define ICM42688_REG_H

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================
 *  SPI 读写标志
 * ============================================================ */
#define ICM42688_SPI_READ           0x80
#define ICM42688_SPI_WRITE          0x00

/* ============================================================
 *  Bank 0 — 核心寄存器
 * ============================================================ */

/* --- WHO_AM_I --- */
#define ICM42688_REG_WHO_AM_I       0x75
#define ICM42688_WHO_AM_I_VALUE     0x47

/* --- 电源/复位 --- */
#define ICM42688_REG_DEVICE_CONFIG   0x11
#define ICM42688_BIT_SOFT_RESET      0x01  /* bit0: 软件复位 */

/* --- 驱动配置 --- */
#define ICM42688_REG_DRIVE_CONFIG    0x13

/* --- 电源管理 --- */
#define ICM42688_REG_PWR_MGMT0       0x6E
#define ICM42688_BIT_TEMP_DIS        0x20  /* bit5: 关闭温度传感器 */
#define ICM42688_BITS_ACCEL_MODE     0x0C  /* bit[3:2]: 加速度计模式 */
#define ICM42688_BITS_GYRO_MODE      0x03  /* bit[1:0]: 陀螺仪模式 */

/* 加速度计量程 */
#define ICM42688_REG_ACCEL_CONFIG0   0x52
#define ICM42688_BITS_ACCEL_FS       0x1C  /* bit[4:2] */
#define ICM42688_BITS_ACCEL_ODR      0x07  /* bit[2:0] — 实际在 bit[6:3] */

/* 陀螺仪量程 */
#define ICM42688_REG_GYRO_CONFIG0    0x51
#define ICM42688_BITS_GYRO_FS        0x1C  /* bit[4:2] */
#define ICM42688_BITS_GYRO_ODR       0x07  /* bit[2:0] — 实际在 bit[6:3] */

/* --- FIFO 配置 --- */
#define ICM42688_REG_FIFO_CONFIG     0x16
#define ICM42688_REG_FIFO_CONFIG1    0x5F
#define ICM42688_REG_FIFO_CONFIG2    0x60
#define ICM42688_REG_FIFO_COUNT      0x2C  /* FIFO 计数 (2 bytes, big-endian) */
#define ICM42688_REG_FIFO_DATA       0x30  /* FIFO 数据读取 */

/* --- 数据就绪 / 中断 --- */
#define ICM42688_REG_INT_SOURCE0     0x65
#define ICM42688_REG_INT_SOURCE1     0x66
#define ICM42688_REG_INT_SOURCE3     0x68
#define ICM42688_REG_INT_SOURCE4     0x69

#define ICM42688_REG_INT_CONFIG      0x14
#define ICM42688_BIT_INT1_POLARITY   0x01  /* bit0: 1=active-high */
#define ICM42688_BIT_INT1_PUSH_PULL  0x00  /* bit1: 0=push-pull */
#define ICM42688_BIT_INT1_LATCH      0x00  /* bit2: 0=不锁存 */

#define ICM42688_REG_INT_CONFIG0     0x63
#define ICM42688_REG_INT_CONFIG1     0x64
#define ICM42688_BIT_INT2_POLARITY   0x04  /* bit2 */
#define ICM42688_BIT_INT_ASYNC_RESET 0x04  /* bit2 in INT_CONFIG1 */

/* --- FIFO 抽取/时间戳 --- */
#define ICM42688_REG_TMST_CONFIG     0x54
#define ICM42688_REG_APEX_CONFIG0    0x56

/* --- 温度数据 (2 bytes) --- */
#define ICM42688_REG_TEMP_DATA1      0x1D  /* 高字节 */
#define ICM42688_REG_TEMP_DATA0      0x1E  /* 低字节 */

/* --- 加速度计数据 (6 bytes) --- */
#define ICM42688_REG_ACCEL_DATA_X1   0x1F  /* X 高字节 */
#define ICM42688_REG_ACCEL_DATA_X0   0x20  /* X 低字节 */
#define ICM42688_REG_ACCEL_DATA_Y1   0x21
#define ICM42688_REG_ACCEL_DATA_Y0   0x22
#define ICM42688_REG_ACCEL_DATA_Z1   0x23
#define ICM42688_REG_ACCEL_DATA_Z0   0x24

/* --- 陀螺仪数据 (6 bytes) --- */
#define ICM42688_REG_GYRO_DATA_X1    0x25  /* X 高字节 */
#define ICM42688_REG_GYRO_DATA_X0    0x26  /* X 低字节 */
#define ICM42688_REG_GYRO_DATA_Y1    0x27
#define ICM42688_REG_GYRO_DATA_Y0    0x28
#define ICM42688_REG_GYRO_DATA_Z1    0x29
#define ICM42688_REG_GYRO_DATA_Z0    0x2A

/* --- 加速度计偏移校准 --- */
#define ICM42688_REG_OFFSET_X_HI     0x77  /* AX_X_OFFSET[15:8] */
#define ICM42688_REG_OFFSET_X_LO     0x78  /* AX_X_OFFSET[7:0]  */
#define ICM42688_REG_OFFSET_Y_HI     0x79
#define ICM42688_REG_OFFSET_Y_LO     0x7A
#define ICM42688_REG_OFFSET_Z_HI     0x7B
#define ICM42688_REG_OFFSET_Z_LO     0x7C

/* --- self-test --- */
#define ICM42688_REG_SELF_TEST       0x6D

/* ============================================================
 *  Bank 1 — FIFO 抽取配置
 * ============================================================ */
#define ICM42688_REG_FIFO_AXES_EN0   0x66  /* 在 bank1 */

/* ============================================================
 *  Bank 2 — 传感器数据噪声配置
 * ============================================================ */

/* ============================================================
 *  Bank 4 — 步数检测 / APEX
 * ============================================================ */
#define ICM42688_REG_APEX_CONFIG1    0x57
#define ICM42688_REG_APEX_CONFIG2    0x58
#define ICM42688_REG_APEX_CONFIG3    0x59
#define ICM42688_REG_APEX_CONFIG4    0x5A
#define ICM42688_REG_APEX_CONFIG5    0x5B
#define ICM42688_REG_APEX_CONFIG6    0x5C
#define ICM42688_REG_APEX_CONFIG7    0x5D
#define ICM42688_REG_APEX_CONFIG8    0x5E
#define ICM42688_REG_APEX_CONFIG9    0x5F
#define ICM42688_REG_APEX_CONFIG10   0x60
#define ICM42688_REG_APEX_CONFIG11   0x61
#define ICM42688_REG_APEX_CONFIG12   0x62

/* ============================================================
 *  Bank selection
 * ============================================================ */
#define ICM42688_REG_REG_BANK_SEL    0x76
#define ICM42688_BANK0               0x00
#define ICM42688_BANK1               0x01
#define ICM42688_BANK2               0x02
#define ICM42688_BANK4               0x04

/* ============================================================
 *  加速度计量程枚举 (写入 ACCEL_CONFIG0[4:2])
 * ============================================================ */
typedef enum {
    ICM42688_ACCEL_16G  = 0x00,  /* ±16g, sensitivity 2048 LSB/g */
    ICM42688_ACCEL_8G   = 0x04,  /* ±8g,  sensitivity 4096 LSB/g */
    ICM42688_ACCEL_4G   = 0x08,  /* ±4g,  sensitivity 8192 LSB/g */
    ICM42688_ACCEL_2G   = 0x0C,  /* ±2g,  sensitivity 16384 LSB/g */
} icm42688_accel_fs_t;

/* ============================================================
 *  陀螺仪量程枚举 (写入 GYRO_CONFIG0[4:2])
 *  灵敏度单位: LSB/(dps)
 * ============================================================ */
typedef enum {
    ICM42688_GYRO_2000DPS  = 0x00,  /* ±2000 dps, 16.4 LSB/dps */
    ICM42688_GYRO_1000DPS  = 0x04,  /* ±1000 dps, 32.8 LSB/dps */
    ICM42688_GYRO_500DPS   = 0x08,  /* ±500  dps, 65.5 LSB/dps */
    ICM42688_GYRO_250DPS   = 0x0C,  /* ±250  dps, 131  LSB/dps */
    ICM42688_GYRO_125DPS   = 0x10,  /* ±125  dps, 262  LSB/dps */
    ICM42688_GYRO_62_5DPS  = 0x14,  /* ±62.5 dps, 524.3 LSB/dps */
    ICM42688_GYRO_31_25DPS = 0x18,  /* ±31.25dps, 1048.6 LSB/dps */
    ICM42688_GYRO_15_625DPS= 0x1C,  /* ±15.625dps, 2097.2 LSB/dps */
} icm42688_gyro_fs_t;

/* ============================================================
 *  ODR (输出数据率) 枚举
 *  ACCEL_CONFIG0[6:3] / GYRO_CONFIG0[6:3]
 * ============================================================ */
typedef enum {
    ICM42688_ODR_1HZ       = 0x03,
    ICM42688_ODR_6HZ       = 0x04,
    ICM42688_ODR_12HZ      = 0x05,
    ICM42688_ODR_25HZ      = 0x06,
    ICM42688_ODR_50HZ      = 0x07,
    ICM42688_ODR_100HZ     = 0x08,
    ICM42688_ODR_200HZ     = 0x09,
    ICM42688_ODR_500HZ     = 0x0A,
    ICM42688_ODR_1000HZ    = 0x0B,
    ICM42688_ODR_2000HZ    = 0x0C,
    ICM42688_ODR_4000HZ    = 0x0D,
    ICM42688_ODR_8000HZ    = 0x0E,
    ICM42688_ODR_NORM_TMR  = 0x0F,  /* 使用定时器 ODR */
} icm42688_odr_t;

/* ============================================================
 *  电源模式
 * ============================================================ */
typedef enum {
    ICM42688_MODE_OFF      = 0x00,
    ICM42688_MODE_STANDBY  = 0x01,
    ICM42688_MODE_LOWNOISE = 0x03,
    ICM42688_MODE_LOWPOWER = 0x02,
} icm42688_power_mode_t;

#ifdef __cplusplus
}
#endif

#endif /* ICM42688_REG_H */
