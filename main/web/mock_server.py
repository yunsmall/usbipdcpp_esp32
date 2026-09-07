#!/usr/bin/env python3
"""本地预览固件内嵌网页（main/web/index.html）用，跑起来后浏览器开 http://127.0.0.1:8000

页面靠 fetch /api/status、/api/devices 拿数据，本脚本返回下面的假数据，
调布局/样式/文案不用烧写固件。想预览"未连接""配置口未启用""超长 SSID"
等状态，改下面的 STATUS/DEVICES 值即可（改动即时生效，刷新页面即可）。
"""
import json
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

PORT = 8000
BASE_DIR = Path(__file__).resolve().parent

# 预览数据：按需修改
STATUS = {
    "connected": True,
    "ssid": "MyHomeWiFi_5G_Example",  # 填长字符串可看 SSID 换行效果
    "ip": "192.168.1.100",
    "console_tx": 17,                 # 改成 -1 预览"配置口未启用"提示块
    "console_rx": 18,
}
DEVICES = {
    "devices": [
        {"busid": "1-1", "vid": "046d", "pid": "c077", "in_use": False},
        {"busid": "1-2", "vid": "058f", "pid": "6387", "in_use": True},
    ],
}


class Handler(BaseHTTPRequestHandler):
    def _send_json(self, obj):
        body = json.dumps(obj, ensure_ascii=False).encode("utf-8")
        self.send_response(200)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        if self.path.startswith("/api/status"):
            self._send_json(STATUS)
        elif self.path.startswith("/api/devices"):
            self._send_json(DEVICES)
        else:
            # 其余路径一律回 index.html（含浏览器可能请求的 /favicon.ico）
            html = (BASE_DIR / "index.html").read_bytes()
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(html)))
            self.end_headers()
            self.wfile.write(html)

    def do_POST(self):
        # 保存按钮的 mock：只回成功；想模拟重连把 STATUS["connected"] 翻掉再点
        self._send_json({"result": "ok, reconnecting"})

    def log_message(self, fmt, *args):
        # 静默访问日志：页面 5 秒轮询两个接口，打了会刷屏
        pass


if __name__ == "__main__":
    server = ThreadingHTTPServer(("127.0.0.1", PORT), Handler)
    print(f"mock 服务已启动：http://127.0.0.1:{PORT} （Ctrl-C 退出）")
    server.serve_forever()
