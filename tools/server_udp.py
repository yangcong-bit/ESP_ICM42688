#!/usr/bin/env python3
"""
ICM-42688 IMU 数据接收服务器

接收 ESP32-S3 通过 UDP 发送的二进制 IMU 数据包,
解包后打印并可选保存到 CSV 文件。

使用方法:
  python server_udp.py                  # 默认监听 0.0.0.0:8888
  python server_udp.py --port 9999      # 自定义端口
  python server_udp.py --save data.csv  # 保存到 CSV
"""

import socket
import struct
import argparse
import time
import csv
import sys

# 数据包格式 (与 net_imu_packet_t 一致):
#   float accel[3]   = 12 bytes
#   float gyro[3]    = 12 bytes
#   float temp       =  4 bytes
#   float quat[4]    = 16 bytes
#   float euler[3]   = 12 bytes
#   uint64 timestamp =  8 bytes
#   Total = 64 bytes
PACKET_FMT = '<3f 3f f 4f 3f Q'
PACKET_SIZE = struct.calcsize(PACKET_FMT)

def unpack_packet(data):
    """解包二进制 IMU 数据包"""
    if len(data) != PACKET_SIZE:
        return None
    vals = struct.unpack(PACKET_FMT, data)
    return {
        'accel':  list(vals[0:3]),
        'gyro':   list(vals[3:6]),
        'temp':   vals[6],
        'quat':   list(vals[7:11]),
        'euler':  list(vals[11:14]),
        'ts':     vals[14],
    }

def main():
    parser = argparse.ArgumentParser(description='ICM-42688 UDP 数据接收器')
    parser.add_argument('--host', default='0.0.0.0', help='监听地址 (默认 0.0.0.0)')
    parser.add_argument('--port', type=int, default=8888, help='监听端口 (默认 8888)')
    parser.add_argument('--save', default=None, help='保存到 CSV 文件路径')
    parser.add_argument('--quiet', action='store_true', help='仅保存, 不打印')
    args = parser.parse_args()

    # 创建 UDP socket
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind((args.host, args.port))
    print(f"[*] 监听 UDP {args.host}:{args.port}")
    print(f"[*] 等待 ESP32 数据包 ({PACKET_SIZE} bytes)...")
    print(f"[*] 按 Ctrl+C 停止\n")

    csv_file = None
    csv_writer = None
    if args.save:
        csv_file = open(args.save, 'w', newline='')
        csv_writer = csv.writer(csv_file)
        csv_writer.writerow([
            'time_s', 'ax', 'ay', 'az',
            'gx', 'gy', 'gz', 'temp',
            'qw', 'qx', 'qy', 'qz',
            'roll', 'pitch', 'yaw', 'timestamp_us'
        ])
        print(f"[*] 保存到: {args.save}")

    count = 0
    t_start = time.time()
    try:
        while True:
            data, addr = sock.recvfrom(1024)
            pkt = unpack_packet(data)
            if pkt is None:
                continue

            count += 1
            t_now = time.time() - t_start

            if not args.quiet:
                print(f"[{count:6d}] {addr[0]:>15s} | "
                      f"A=[{pkt['accel'][0]:+7.3f}, {pkt['accel'][1]:+7.3f}, {pkt['accel'][2]:+7.3f}] g | "
                      f"G=[{pkt['gyro'][0]:+7.1f}, {pkt['gyro'][1]:+7.1f}, {pkt['gyro'][2]:+7.1f}] dps | "
                      f"E=[{pkt['euler'][0]:+7.1f}°, {pkt['euler'][1]:+7.1f}°, {pkt['euler'][2]:+7.1f}°] | "
                      f"T={pkt['temp']:.1f}°C")

            if csv_writer:
                csv_writer.writerow([
                    f"{t_now:.3f}",
                    *[f"{v:.4f}" for v in pkt['accel']],
                    *[f"{v:.2f}" for v in pkt['gyro']],
                    f"{pkt['temp']:.1f}",
                    *[f"{v:.5f}" for v in pkt['quat']],
                    *[f"{v:.2f}" for v in pkt['euler']],
                    pkt['ts']
                ])
                if count % 50 == 0:
                    csv_file.flush()

    except KeyboardInterrupt:
        print(f"\n[*] 收到 {count} 个数据包, 运行 {time.time()-t_start:.1f}s")
    finally:
        if csv_file:
            csv_file.close()
        sock.close()

if __name__ == '__main__':
    main()
