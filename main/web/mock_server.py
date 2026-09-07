#!/usr/bin/env python3
"""本地预览固件内嵌网页（main/web/index.html）用，跑起来后浏览器开 http://127.0.0.1:8000

行为模拟固件 HttpConfigApi/WifiConfigManager：
- GET  /api/status：当前状态（改下方 state 可预览不同初始状态）
- GET  /api/devices：设备列表（增删条目看表格效果）
- POST /api/wifi：与固件一致的校验（ssid 必填、32/64 长度上限）与流程——
  先应答 200，随即"断开"（connected=false、ip 清空，页面红点），约 2.5 秒后
  模拟重连成功恢复绿点。密码以 "bad" 开头时模拟凭据错误：保持断开不恢复，
  方便预览"连不上"的页面表现，再次保存即可恢复。
"""
import json
import threading
import urllib.parse
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

PORT = 8000
BASE_DIR = Path(__file__).resolve().parent
MAX_FORM_BODY = 512  # 与 HttpConfigApi.cpp 一致

# 初始状态：按需修改（长 SSID 看换行；console_tx=-1 看"配置口未启用"提示）
state = {
    "connected": True,
    "ssid": "MyHomeWiFi_5G_Example",
    "ip": "192.168.1.100",
    "console_tx": 17,
    "console_rx": 18,
}
devices = {
    "devices": [
        {"busid": "1-1", "vid": "046d", "pid": "c077", "in_use": False},
        {"busid": "1-2", "vid": "058f", "pid": "6387", "in_use": True},
    ],
}

# 最近一次保存的密码：模拟重连结果用（见 POST /api/wifi）
last_password = ""


def schedule_reconnect(ssid: str) -> None:
    """模拟固件 apply_config 后的重连：断开约 2.5 秒后连上新网恢复状态。
    密码以 bad 开头 = 凭据错误，保持未连接直到下次保存"""
    def reconnect():
        if last_password.lower().startswith("bad"):
            return  # 模拟密码错误：页面持续显示未连接
        state["connected"] = True
        state["ssid"] = ssid
        state["ip"] = "192.168.1.100"

    timer = threading.Timer(2.5, reconnect)
    timer.daemon = True
    timer.start()


class Handler(BaseHTTPRequestHandler):
    def _send_json(self, obj, code=200):
        body = json.dumps(obj, ensure_ascii=False).encode("utf-8")
        self.send_response(code)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        if self.path.startswith("/api/status"):
            self._send_json(state)
        elif self.path.startswith("/api/devices"):
            self._send_json(devices)
        else:
            # 其余路径一律回 index.html（含浏览器可能请求的 /favicon.ico）
            html = (BASE_DIR / "index.html").read_bytes()
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(html)))
            self.end_headers()
            self.wfile.write(html)

    def do_POST(self):
        if not self.path.startswith("/api/wifi"):
            return self._send_json({"error": "not found"}, 404)

        # 与固件相同的 body 读取与上限
        length = int(self.headers.get("Content-Length") or 0)
        body = self.rfile.read(length).decode("utf-8", "replace")
        if length <= 0 or length > MAX_FORM_BODY:
            return self._send_json({"error": "body too large or empty"}, 400)

        # 与固件相同的预检（HttpConfigApi.cpp）：ssid 必填、字节长度上限
        form = urllib.parse.parse_qs(body, keep_blank_values=True)
        ssid = (form.get("ssid", [""])[0]).strip()
        password = form.get("password", [""])[0]
        if not ssid:
            return self._send_json({"error": "ssid required"}, 400)
        if len(ssid.encode("utf-8")) >= 32 or len(password.encode("utf-8")) >= 64:
            return self._send_json({"error": "ssid/password too long"}, 400)

        # 固件流程：应答已由返回语句发出，这里先置断开状态再排重连
        global last_password
        last_password = password
        state["connected"] = False
        state["ip"] = ""
        self._send_json({"result": "ok, reconnecting"})
        print(f"[mock] 保存 WiFi: ssid={ssid!r} password={'***' if password else '(空=开放网络)'}"
              f"{'（bad 开头，模拟连不上）' if password.lower().startswith('bad') else ''}")
        schedule_reconnect(ssid)

    def log_message(self, fmt, *args):
        # 静默访问日志：页面 5 秒轮询两个接口，打了会刷屏
        pass


if __name__ == "__main__":
    server = ThreadingHTTPServer(("127.0.0.1", PORT), Handler)
    print(f"mock 服务已启动：http://127.0.0.1:{PORT} （Ctrl-C 退出）")
    server.serve_forever()
