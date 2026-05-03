#!/usr/bin/env python3
"""
ICM-42688 IMU HTTP 数据接收服务器

接收 ESP32-S3 通过 HTTP POST 发送的 JSON IMU 数据。

使用方法:
  python server_http.py                  # 默认监听 0.0.0.0:8080
  python server_http.py --port 9999      # 自定义端口
"""

import json
import argparse
from http.server import HTTPServer, BaseHTTPRequestHandler

class IMUHandler(BaseHTTPRequestHandler):
    def do_POST(self):
        content_length = int(self.headers.get('Content-Length', 0))
        body = self.rfile.read(content_length)

        try:
            pkt = json.loads(body.decode('utf-8'))
            ts = pkt.get('ts', 0)
            e = pkt.get('e', [0, 0, 0])
            a = pkt.get('a', [0, 0, 0])
            g = pkt.get('g', [0, 0, 0])
            t = pkt.get('t', 0)

            print(f"IMU | A=[{a[0]:+7.3f}, {a[1]:+7.3f}, {a[2]:+7.3f}] g | "
                  f"G=[{g[0]:+7.1f}, {g[1]:+7.1f}, {g[2]:+7.1f}] dps | "
                  f"E=[{e[0]:+7.1f}°, {e[1]:+7.1f}°, {e[2]:+7.1f}°] | "
                  f"T={t:.1f}°C | ts={ts}")

            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.end_headers()
            self.wfile.write(b'{"ok":true}')
        except Exception as ex:
            print(f"Error: {ex}")
            self.send_response(400)
            self.end_headers()
            self.wfile.write(b'{"error":"bad request"}')

    def log_message(self, format, *args):
        pass  # 静默 HTTP 日志

def main():
    parser = argparse.ArgumentParser(description='ICM-42688 HTTP 数据接收器')
    parser.add_argument('--host', default='0.0.0.0', help='监听地址')
    parser.add_argument('--port', type=int, default=8080, help='监听端口')
    args = parser.parse_args()

    server = HTTPServer((args.host, args.port), IMUHandler)
    print(f"[*] HTTP 服务器监听 {args.host}:{args.port}")
    print(f"[*] ESP32 POST 目标: http://<ESP32_IP>:8080/api/imu")
    print(f"[*] 按 Ctrl+C 停止\n")

    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\n[*] 服务器已停止")
        server.server_close()

if __name__ == '__main__':
    main()
