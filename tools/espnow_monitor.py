#!/usr/bin/env python3
"""
ESP-NOW IMU 数据 Python 接收端

配合 espnow_receiver 示例使用, 通过串口读取 ESP32 接收到的 ESP-NOW 数据。

使用方法:
  1. 将 espnow_receiver 烧录到 ESP32 (接收端)
  2. ESP32 通过 USB 串口连接电脑
  3. 运行此脚本: python espnow_monitor.py --port COM3

注意: 此脚本读取的是串口日志, 不是直接解析 ESP-NOW 帧。
如需直接解析 ESP-NOW 帧, 需要使用 ESP-IDF 的 WiFi sniffer 模式。
"""

import serial
import time
import argparse
import re

def main():
    parser = argparse.ArgumentParser(description='ESP-NOW IMU 串口监控')
    parser.add_argument('--port', default='COM3', help='串口号 (默认 COM3)')
    parser.add_argument('--baud', type=int, default=115200, help='波特率 (默认 115200)')
    parser.add_argument('--save', default=None, help='保存日志到文件')
    args = parser.parse_args()

    try:
        ser = serial.Serial(args.port, args.baud, timeout=1)
        print(f"[*] 已连接 {args.port} @ {args.baud} baud")
    except Exception as e:
        print(f"[!] 串口连接失败: {e}")
        print(f"    请检查端口号, 或安装 pyserial: pip install pyserial")
        return

    log_file = None
    if args.save:
        log_file = open(args.save, 'w', encoding='utf-8')
        print(f"[*] 日志保存到: {args.save}")

    print(f"[*] 等待 ESP-NOW 数据...\n")

    try:
        while True:
            line = ser.readline().decode('utf-8', errors='ignore').strip()
            if not line:
                continue

            # 过滤包含 IMU 数据的行
            if 'R=' in line and 'P=' in line and 'Y=' in line:
                ts = time.strftime('%H:%M:%S')
                print(f"[{ts}] {line}")

                if log_file:
                    log_file.write(f"{ts} {line}\n")
                    log_file.flush()

    except KeyboardInterrupt:
        print(f"\n[*] 监控已停止")
    finally:
        ser.close()
        if log_file:
            log_file.close()

if __name__ == '__main__':
    main()
