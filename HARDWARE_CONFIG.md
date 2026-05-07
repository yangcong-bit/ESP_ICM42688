# ESP32-S3 硬件配置说明

## 当前硬件规格
- **Flash**: 16MB
- **PSRAM**: 8MB (Octal)

> ⚠️ N16R8 模组必须使用 Octal (八线 OPI) 模式，否则会触发 quad_psram 初始化失败。

## 已修改的配置

### 1. Flash 大小 (sdkconfig)
```ini
CONFIG_ESPTOOLPY_FLASHSIZE_16MB=y
CONFIG_ESPTOOLPY_FLASHSIZE="16MB"
```

### 2. PSRAM 配置 (sdkconfig)
```ini
CONFIG_SPIRAM=y
CONFIG_SPIRAM_MODE_OCT=y           # Octal 模式 (八线 OPI)
CONFIG_SPIRAM_SPEED_80M=y          # 80MHz 频率
CONFIG_SPIRAM_BOOT_INIT=y          # 启动时初始化
CONFIG_SPIRAM_MALLOC_ALWAYSINTERNAL=16384
CONFIG_SPIRAM_TRY_ALLOCATE_WIFI_LWIP=y
CONFIG_SPIRAM_ALLOW_BSS_SEG_EXTERNAL_MEMORY=y
```

### 3. 分区表 (partitions_16mb.csv)
| 分区 | 类型 | 大小 | 说明 |
|------|------|------|------|
| nvs | data | 24KB | 非易失性存储 |
| phy_init | data | 4KB | PHY 初始化数据 |
| factory | app | 6MB | 应用程序 |
| storage | data | spiffs | 12MB | SPIFFS 文件系统 |

## 应用配置

### 方法 1: 重新生成 sdkconfig (推荐)
```bash
idf.py reconfigure
```

### 方法 2: 清除并重新配置
```bash
idf.py fullclean
idf.py reconfigure
```

### 方法 3: 使用 menuconfig
```bash
idf.py menuconfig
```
然后手动调整:
- Component config → ESP PSRAM → 启用 PSRAM
- Serial flasher config → Flash size → 16MB
- Partition Table → Custom partition table → partitions_16mb.csv

## 验证配置

### 检查 Flash 大小
```bash
idf.py size
```

### 检查 PSRAM
烧录后查看串口输出:
```
I (xxx) spiram: Found 8MB Octal PSRAM
```

## 注意事项

1. **PSRAM 用于**: 
   - 大型缓冲区
   - WiFi/BT 协议栈
   - 需要大内存的应用

2. **Flash 用于**:
   - 程序存储
   - NVS (配置存储)
   - SPIFFS (文件系统)

3. **性能优化**:
   - PSRAM 访问速度较慢 (80MHz vs Flash 80MHz)
   - 关键数据尽量放在内部 SRAM
   - 使用 `heap_caps_malloc(MALLOC_CAP_SPIRAM)` 分配 PSRAM 内存

## 常见问题

### Q: PSRAM 容量显示不正确?
A: N16R8 的 PSRAM 为 8MB Octal (八线)。请务必检查是否已正确开启 CONFIG_SPIRAM_MODE_OCT=y，切勿使用 Quad 模式，否则会触发 quad_psram 初始化失败。

### Q: Flash 烧录失败?
A: 确认使用正确的 Flash 大小参数:
```bash
idf.py flash --flash_size 8MB
```

### Q: 内存不足?
A: 检查内存使用:
```bash
idf.py size-components
```
考虑将不常用数据移到 PSRAM
