# ESP32-S3 双 ICM-42688 融合姿态解算系统

基于 ESP32-S3 + 双路 ICM-42688-P IMU 的实时姿态解算系统，支持 ESP-NOW 无线广播、中断驱动读取、全局时钟同步。

## ✨ 功能特性

| 模块 | 功能 |
|------|------|
| **ICM42688 SPI 驱动** | 初始化、burst read 14B、量程配置、偏移校准 |
| **中断驱动读取** | INT1 → GPIO 中断 → 二值信号量 → 高优先级任务唤醒，CPU 零空转 |
| **6-ESKF 姿态解算** | 6状态误差卡尔曼滤波，序贯更新零矩阵求逆，医疗级精度 |
| **双路融合** | 交叉校准、安装误差自动补偿、异常检测、单路降级运行 |
| **ESP-NOW 广播** | 无需路由器、低延迟 (~2ms)、30~200m 通信距离 |
| **全局时钟同步** | 主机广播 offset 校正，多节点微秒级时间对齐 |
| **IRAM 优化** | 关闭无用调试选项，释放 IRAM 给 ESP-NOW/WiFi 协议栈 |

## 📁 项目结构

```
ESP_ICM42688/
├── CMakeLists.txt                    # 顶层项目构建
├── README.md
├── HARDWARE_CONFIG.md                # 硬件配置说明 (8MB Flash / 16MB PSRAM)
├── partitions_8mb.csv                # 自定义分区表 (3MB app + 5MB storage)
├── sdkconfig.defaults                # 含 IRAM 优化配置
├── components/
│   ├── icm42688/                     # ICM-42688 驱动 + 解算库
│   │   ├── include/
│   │   │   ├── icm42688.h            # SPI 驱动 + 中断 API
│   │   │   ├── icm42688_reg.h        # 寄存器定义 & 量程枚举
│   │   │   ├── icm42688_alg.h        # 四元数/欧拉角工具 + 滤波器
│   │   │   ├── icm42688_dual.h       # 双 IMU 融合 + 3x3 矩阵运算
│   │   │   └── hardcore_eskf.h       # 6状态 ESKF 滤波器
│   │   └── src/
│   │       ├── icm42688.c            # SPI 驱动 + 中断 ISR
│   │       ├── icm42688_alg.c        # 四元数/欧拉角工具
│   │       ├── icm42688_dual.c       # 双路融合 (调用 ESKF)
│   │       └── hardcore_eskf.c       # ESKF 核心 (IRAM_ATTR)
│   └── net_send/                     # 网络发送 + 时间同步组件
│       ├── include/
│       │   ├── net_send.h            # ESP-NOW 广播 API
│       │   └── time_sync.h           # 全局时钟同步协议
│       └── src/
│           ├── net_send.c            # ESP-NOW 广播 + 同步包处理
│           └── time_sync.c           # 三步时钟同步实现
├── main/
│   ├── CMakeLists.txt
│   └── main.c                        # 主程序: 中断驱动 + 双 IMU + ESP-NOW
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

| 信号 | GPIO | 说明 |
|------|------|------|
| SCLK | 12 | SPI 时钟 |
| MOSI | 11 | 主出从入 |
| MISO | 13 | 主入从出 |
| CS | 10 | 片选 |
| **INT1** | **46** | **Data Ready 中断输出** |

### IMU-B (SPI3)

| 信号 | GPIO | 说明 |
|------|------|------|
| SCLK | 36 | SPI 时钟 |
| MOSI | 35 | 主出从入 |
| MISO | 37 | 主入从出 |
| CS | 34 | 片选 |
| **INT1** | **9** | **Data Ready 中断输出** |

> 如果两路 SPI 共享总线，可共用 SCLK/MOSI/MISO，仅需不同 CS 引脚。INT 引脚必须独立连接到各自的 GPIO。

## 🚀 快速开始

### 1. 环境准备

- [ESP-IDF v5.5+](https://docs.espressif.com/projects/esp-idf/zh_CN/latest/esp32s3/get-started/)
- VS Code + ESP-IDF 扩展（推荐）

### 2. 配置引脚

编辑 `main/main.c` 顶部的宏定义：

```c
#define PIN_SCLK_A    12
#define PIN_MOSI_A    11
#define PIN_MISO_A    13
#define PIN_CS_A      10
#define PIN_INT_A     46     /* IMU-A 中断引脚 */
// ... IMU-B 类似
#define NODE_ID       0x01   /* 节点 ID (时间同步用, 每个节点唯一) */
```

### 3. 编译烧录

```bash
idf.py set-target esp32s3
idf.py reconfigure     # 应用 sdkconfig.defaults (IRAM 优化等)
idf.py build
idf.py -p COMx flash monitor
```

### 4. 接收数据

将 `examples/espnow_receiver/` 烧录到另一块 ESP32：

```bash
cd examples/espnow_receiver
idf.py set-target esp32s3
idf.py build flash monitor
```

## 🧠 架构设计

### 系统架构

```
                    ┌─────────────────────────────────────┐
                    │           ESP32-S3 主节点            │
  ICM42688-A ─SPI──→│  INT1(46) ─→ GPIO ISR ─→ 信号量     │
                    │  ┌─────────────────────────────┐    │
  ICM42688-B ─SPI──→│  │  高优先级读取任务            │    │──→ ESP-NOW 广播
  INT1(9) ─────────→│  │  wait_drdy → burst read     │    │    (64B 二进制包)
                    │  │  → 双路融合 → 6-ESKF 解算   │    │
                    │  └─────────────────────────────┘    │
                    │  时间同步: 收到 SYNC_START 自动回复  │
                    └─────────────────────────────────────┘
                                       │
                         ESP-NOW 广播 (同信道所有设备)
                                       │
                    ┌──────────────────┼──────────────────┐
                    │                  │                   │
               ┌────▼────┐       ┌─────▼────┐       ┌─────▼────┐
               │ 接收端1  │       │ 接收端2   │       │ 接收端N  │
               │ ESP32   │       │ RK3566   │       │ PC      │
               └─────────┘       └──────────┘       └─────────┘
```

### 中断驱动 vs 轮询

| | 轮询 (旧) | 中断驱动 (新) |
|---|---|---|
| **CPU 占用** | ~100% (vTaskDelay 忙等) | ~5% (信号量阻塞让出 CPU) |
| **延迟** | 取决于 vTaskDelay 周期 | 中断触发后 ~10μs 响应 |
| **功耗** | 高 (CPU 持续运行) | 低 (CPU 睡眠等待中断) |
| **实时性** | 受其他任务干扰 | 由硬件中断保证 |

### 双路融合流程

```
IMU-A ─→ 读取 ─→ [坐标变换] ──┐
                                ├──→ 加权融合 ──→ 6-ESKF 解算 ──→ 四元数/欧拉角
IMU-B ─→ 读取 ─→ [安装补偿] ──┘        ↑            ↓
                                交叉校验     eskf_predict (预测步)
                                & 异常检测   eskf_update_accel (更新步)
```

### 6状态 ESKF 算法

采用 **Error-State Kalman Filter**（误差状态卡尔曼滤波器），相比传统 Mahony 互补滤波器具有更高精度和更强的噪声抑制能力：

| 特性 | 说明 |
|------|------|
| **状态维度** | 6维：角度误差(3) + 陀螺仪零偏(3) |
| **预测步** | 陀螺仪积分更新名义四元数，协方差矩阵 F 传播 |
| **更新步** | 加速度计序贯更新，3次标量运算替代矩阵求逆 |
| **零偏估计** | 实时估计并补偿陀螺仪零偏漂移 |
| **IRAM 优化** | 核心运算函数标记 `IRAM_ATTR`，确保中断级实时性 |
| **零动态内存** | 全部 6×6 矩阵运算在栈上完成，无 malloc |

**调参指南**：
- `noise_gyro` (默认 0.001)：陀螺仪白噪声方差，增大则更信任加速度计
- `noise_bias` (默认 0.0001)：零偏随机游走方差，增大则零偏收敛更快
- `noise_accel` (默认 0.05)：加速度计噪声方差，增大则加速度修正更保守

### 全局时钟同步协议

```
主机 (RK3566)              节点 (ESP32)
    │                          │
    │──── SYNC_START ─────────→│  T_host_send
    │                          │
    │←─── SYNC_REPLY ──────────│  T_node_rx
    │                          │
    │  计算 RTT = T_host_rx - T_host_send
    │  offset = (T_host_send + T_host_rx) / 2 - T_node_rx
    │                          │
    │──── SYNC_APPLY ─────────→│  应用 offset
    │                          │
    │  (每秒重复, 精度 ~10-50μs) │
```

## 📡 数据协议

### ESP-NOW 广播数据包（64 字节，小端序）

| 偏移 | 类型 | 字段 | 说明 |
|------|------|------|------|
| 0~11 | float×3 | accel | 加速度 (g) |
| 12~23 | float×3 | gyro | 陀螺仪 (dps) |
| 24~27 | float | temp | 温度 (°C) |
| 28~43 | float×4 | quat | 四元数 [w,x,y,z] |
| 44~55 | float×3 | euler | 欧拉角 [roll,pitch,yaw] (°) |
| 56~63 | uint64 | timestamp_us | **全局同步时间戳** (μs) |

### 时间同步包（13 字节）

| 偏移 | 类型 | 字段 | 说明 |
|------|------|------|------|
| 0 | uint8 | type | 0xFD (时间同步专用) |
| 1 | uint8 | node_id | 节点 ID |
| 2~5 | uint32 | seq | 序列号 |
| 6~13 | int64 | host_time_us | 主机时间戳 |
| 14~21 | int64 | node_time_us | 节点时间戳 |
| 22~29 | int64 | offset_us | 时间偏移 |

## 🔧 硬件配置

本项目默认配置：
- **Flash**: 8MB
- **PSRAM**: 16MB (Quad, 80MHz)
- **分区表**: `partitions_8mb.csv` (3MB app + 5MB storage)
- **IRAM 优化**: 关闭 Secure Boot / Core dump / WiFi 日志等

详见 [HARDWARE_CONFIG.md](HARDWARE_CONFIG.md)。

## 📊 资源占用

| 资源 | 使用量 | 说明 |
|------|--------|------|
| Flash | ~561 KB | 代码 + 数据 |
| DIRAM | 31.8% | 含 WiFi/BT 协议栈 |
| IRAM | 100% → 优化后 ~80% | IRAM 优化后释放给 ESP-NOW |

## 📝 串口输出示例

```
=== 双 ICM-42688-P AHRS + ESP-NOW Demo ===
初始化 IMU-A...
初始化 IMU-B...
配置中断驱动读取...
Interrupt configured on GPIO 46 (falling edge, DRDY)
Interrupt configured on GPIO 9 (falling edge, DRDY)
请保持传感器静止, 开始交叉校准...
校准后加速度 RMSE: 0.0089 g
=== 校准完成 ===
ESP-NOW + 时间同步就绪 (node_id=0x01)
=== 开始双路融合解算 (100Hz, 中断驱动) ===

[200] R=  1.2° P= -0.8° Y=  45.3° | Conf=98% | ΔA=0.012g ΔG=2.3dps | TX✓
  IMU-A: ON (IRQ:200) | IMU-B: ON (IRQ:200) | Sync:OK #15
```

## 📜 License

MIT
