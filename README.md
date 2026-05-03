# ESP32-S3 双 ICM-42688 融合姿态解算系统

基于 ESP32-S3 + 双路 ICM-42688-P IMU 的实时姿态解算系统，支持 ESP-NOW 无线广播数据传输。

## ✨ 功能特性

| 模块 | 功能 |
|------|------|
| **ICM42688 SPI 驱动** | 初始化、burst read、量程配置、偏移校准 |
| **Mahony AHRS 解算** | 四元数互补滤波器、欧拉角输出、在线陀螺仪偏置估计 |
| **双路融合** | 交叉校准、安装误差自动补偿、异常检测、降级运行 |
| **ESP-NOW 广播** | 无需路由器、低延迟 (~2ms)、30~200m 通信距离 |
| **WiFi 传输** | UDP / HTTP POST 可切换 |
| **信号滤波** | 滑动平均、一阶 IIR 低通、加权融合 |

## 📁 项目结构

```
ESP_ICM42688/
├── CMakeLists.txt                    # 顶层项目构建
├── README.md
├── HARDWARE_CONFIG.md                # 硬件配置说明 (8MB Flash / 16MB PSRAM)
├── partitions_8mb.csv                # 自定义分区表
├── sdkconfig.defaults
├── components/
│   ├── icm42688/                     # ICM-42688 驱动 + 解算库
│   │   ├── include/
│   │   │   ├── icm42688.h            # SPI 驱动 API
│   │   │   ├── icm42688_reg.h        # 寄存器定义 & 量程枚举
│   │   │   ├── icm42688_alg.h        # Mahony AHRS + 滤波器 API
│   │   │   └── icm42688_dual.h       # 双 IMU 融合 API + 矩阵运算
│   │   └── src/
│   │       ├── icm42688.c            # SPI 驱动实现
│   │       ├── icm42688_alg.c        # AHRS 解算实现
│   │       └── icm42688_dual.c       # 双路融合实现
│   └── net_send/                     # 网络发送组件
│       ├── include/
│       │   └── net_send.h            # ESP-NOW / WiFi API
│       └── src/
│           └── net_send.c            # ESP-NOW 广播实现
├── main/
│   ├── CMakeLists.txt
│   └── main.c                        # 主程序: 双 IMU + ESP-NOW
├── examples/
│   └── espnow_receiver/              # ESP-NOW 接收端示例
│       ├── CMakeLists.txt
│       └── main/main.c
└── tools/
    ├── espnow_monitor.py             # 串口监控脚本
    ├── server_udp.py                 # Python UDP 接收服务器
    └── server_http.py                # Python HTTP 接收服务器
```

## 🔌 硬件连接

### IMU-A (SPI2)

| 信号 | GPIO |
|------|------|
| SCLK | 12 |
| MOSI | 11 |
| MISO | 13 |
| CS | 10 |

### IMU-B (SPI3)

| 信号 | GPIO |
|------|------|
| SCLK | 36 |
| MOSI | 35 |
| MISO | 37 |
| CS | 34 |

> 如果两路 SPI 共享总线，可共用 SCLK/MOSI/MISO，仅需不同 CS 引脚。

## 🚀 快速开始

### 1. 环境准备

- [ESP-IDF v5.5+](https://docs.espressif.com/projects/esp-idf/zh_CN/latest/esp32s3/get-started/)
- VS Code + ESP-IDF 扩展（推荐）

### 2. 配置引脚

编辑 `main/main.c` 顶部的宏定义，匹配你的硬件接线：

```c
#define PIN_SCLK_A    12
#define PIN_MOSI_A    11
#define PIN_MISO_A    13
#define PIN_CS_A      10
// ... IMU-B 类似
```

### 3. 编译烧录

```bash
idf.py set-target esp32s3
idf.py reconfigure
idf.py build
idf.py -p COMx flash monitor
```

### 4. 接收数据

#### 方式一：ESP-NOW 接收（推荐）

将 `examples/espnow_receiver/` 烧录到另一块 ESP32：

```bash
cd examples/espnow_receiver
idf.py set-target esp32s3
idf.py build flash monitor
```

#### 方式二：Python UDP 接收

切换到 WiFi UDP 模式后，在电脑上运行：

```bash
python tools/server_udp.py
python tools/server_udp.py --save imu_data.csv   # 保存到 CSV
```

## 🧠 算法说明

### 双路融合流程

```
IMU-A ─→ 读取 ─→ [坐标变换] ──┐
                                ├──→ 加权融合 ──→ Mahony AHRS ──→ 四元数/欧拉角
IMU-B ─→ 读取 ─→ [安装补偿] ──┘        ↑
                                交叉校验 & 异常检测
```

### 核心算法

| 算法 | 说明 |
|------|------|
| **Mahony AHRS** | 互补滤波器，加速度修正 + 陀螺仪积分，`kp=1.0, ki=0.005` |
| **安装误差补偿** | 自动检测两路传感器安装偏差角（Rodrigues 旋转估算） |
| **交叉校验** | 实时比较两路数据差异，超阈值自动降级到单路 |
| **偏置交叉补偿** | 运行时估计两路陀螺仪偏置差异并修正 |
| **加权融合** | `alpha` 控制权重（0.5 = 等权），可调 |

### 融合参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `alpha` | 0.5 | IMU-A 融合权重（0~1） |
| `kp` | 1.0 | Mahony 比例增益 |
| `ki` | 0.005 | Mahony 积分增益 |
| `sample_hz` | 1000 | IMU 采样率 |
| `enable_misalign` | true | 启用安装误差补偿 |
| `enable_cross_bias` | true | 启用交叉偏置补偿 |

## 📡 数据协议

### ESP-NOW 广播数据包（64 字节，小端序）

| 偏移 | 类型 | 字段 | 说明 |
|------|------|------|------|
| 0~11 | float×3 | accel | 加速度 (g) |
| 12~23 | float×3 | gyro | 陀螺仪 (dps) |
| 24~27 | float | temp | 温度 (°C) |
| 28~43 | float×4 | quat | 四元数 [w,x,y,z] |
| 44~55 | float×3 | euler | 欧拉角 [roll,pitch,yaw] (°) |
| 56~63 | uint64 | timestamp_us | 时间戳 (μs) |

## 📊 资源占用

| 资源 | 使用量 | 说明 |
|------|--------|------|
| Flash | ~561 KB | 代码 + 数据 |
| DIRAM | 31.8% | 含 WiFi/BT 协议栈 |
| IRAM | 100% | 已满，注意后续扩展 |

## 🔧 硬件配置

本项目默认配置：
- **Flash**: 8MB
- **PSRAM**: 16MB (Quad, 80MHz)
- **分区表**: `partitions_8mb.csv` (3MB app + 5MB storage)

详见 [HARDWARE_CONFIG.md](HARDWARE_CONFIG.md)。

## 📝 串口输出示例

```
=== 双 ICM-42688-P AHRS + ESP-NOW Demo ===
初始化 IMU-A...
初始化 IMU-B...
请保持传感器静止, 开始交叉校准...
校准后加速度 RMSE: 0.0089 g
=== 校准完成 ===
初始化 ESP-NOW 广播 (无需路由器)
ESP-NOW 广播就绪, 所有同信道 ESP32 均可接收

[200] R=  1.2° P= -0.8° Y=  45.3° | Conf=98% | ΔA=0.012g ΔG=2.3dps | TX✓
  IMU-A: ON | IMU-B: ON | Cross: OK
```

## 📜 License

MIT
