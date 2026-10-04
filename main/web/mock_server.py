#!/usr/bin/env python3
"""本地预览固件内嵌网页（main/web/index.html）用，跑起来后浏览器开 http://127.0.0.1:8000

行为模拟固件 HttpConfigApi/WifiConfigManager：
- GET  /api/status：当前状态（改下方 state 可预览不同初始状态；把 ap_active 置
  true 可预览配网模式：页面顶部横幅 + WiFi 卡自动展开）
- GET  /api/devices：设备列表（增删条目看表格效果）
- POST /api/wifi：与固件一致的校验（ssid 必填、32/64 长度上限）与流程——同步
  "等连接结果"（sleep 模拟，期间 state 置为断开，页面轮询能看到中间态），连上
  才置成功并应答 {"ok":true,"ip":..}；密码以 "bad" 开头时模拟凭据错误：等一会
  应答 {"ok":false,"code":..}，不保存（state 保持断开前的原样），方便预览
  "连不上"的页面表现，再次保存即可恢复。
- POST /api/ap：改配网热点（ssid 必填、密码留空=开放、至少 8 位），只改配置，
  不动 ap_active（与固件"下次热点启动时才生效"一致）
- POST /api/mode：切期望工作模式 mode=sta|ap，立即生效（切 ap 时一并模拟
  "断开 WiFi + 开热点"）
"""
import json
import time
import urllib.parse
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

PORT = 8000
BASE_DIR = Path(__file__).resolve().parent
MAX_FORM_BODY = 512  # 与 HttpConfigApi.cpp 一致

# 初始状态：按需修改（长 SSID 看换行；console_tx=-1 看"配置口未启用"提示；
# ap_active=True 预览配网模式页面）
# 不做并发保护：仅供单用户本地预览，state 只改值不改结构（Python 层不会抛），
# 最坏是页面恰好轮询到一次中间态——而那本来就是真实 POST 流程里会出现的样子
state = {
    "connected": True,
    "ssid": "MyHomeWiFi_5G_Example",
    "ip": "192.168.1.100",
    "console_tx": 17,
    "console_rx": 18,
    "ap_active": False,
    "ap_ssid": "usbipd-setup",
    "ap_auth": True,
    "wifi_auth": True,
    "work_mode": "sta",
}
devices = {
    "devices": [
        {"busid": "1-1", "vid": "046d", "pid": "c077", "in_use": False},
        {"busid": "1-2", "vid": "058f", "pid": "6387", "in_use": True},
    ],
}


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
        elif self.path.startswith("/api/"):
            # 未知 API 回 JSON 404 而不是同一个 index.html：回 HTML 的话前端
            # .json() 会抛解析错误，看起来像页面坏了，而不是"这个接口不存在"
            self._send_json({"code": "not_found"}, 404)
        else:
            # 其余路径一律回 index.html（含浏览器可能请求的 /favicon.ico）
            html = (BASE_DIR / "index.html").read_bytes()
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(html)))
            self.end_headers()
            self.wfile.write(html)

    def do_POST(self):
        if self.path.startswith("/api/ap"):
            return self._handle_ap()
        if self.path.startswith("/api/mode"):
            return self._handle_mode()
        if not self.path.startswith("/api/wifi"):
            return self._send_json({"code": "not_found"}, 404)

        # 与固件相同的 body 读取与上限
        length = int(self.headers.get("Content-Length") or 0)
        body = self.rfile.read(length).decode("utf-8", "replace")
        if length <= 0 or length > MAX_FORM_BODY:
            return self._send_json({"code": "bad_body"}, 400)

        # 与固件相同的预检（HttpConfigApi.cpp）：ssid 必填、字节长度上限
        form = urllib.parse.parse_qs(body, keep_blank_values=True)
        ssid = (form.get("ssid", [""])[0]).strip()
        password = form.get("password", [""])[0]
        if not ssid:
            return self._send_json({"code": "ssid_required"}, 400)
        if len(ssid.encode("utf-8")) >= 32 or len(password.encode("utf-8")) >= 64:
            return self._send_json({"code": "too_long"}, 400)

        # 固件流程：同步等连接结果（最长 15 秒）才应答——连上才保存，失败不保存。
        # 这里用 sleep 模拟等待，期间页面轮询能看到"断开中"的中间态
        state["connected"] = False
        state["ip"] = ""
        if password.lower().startswith("bad"):
            time.sleep(2.0)
            self._send_json({"ok": False, "code": "ap_rejected"})
            print(f"[mock] 保存 WiFi: ssid={ssid!r}（bad 开头，模拟连不上，未保存）")
            return
        time.sleep(1.5)
        state["connected"] = True
        state["ssid"] = ssid
        state["wifi_auth"] = bool(password)
        state["ip"] = "192.168.1.100"
        # SSID 以 nvs 开头模拟"NVS 写失败"（连得上但保存不了，配合页面的提示分支验证）
        persisted = not ssid.lower().startswith("nvs")
        self._send_json({"ok": True, "ip": state["ip"], "persisted": persisted})
        print(f"[mock] 保存 WiFi: ssid={ssid!r} password={'***' if password else '(空=开放网络)'}"
              f" → 连接成功")

    def _handle_ap(self):
        # 与固件 WifiConfigManager::apply_ap_config 相同的校验：ssid 必填，
        # 密码留空=开放热点，非空则至少 8 位（WPA2 下限）
        length = int(self.headers.get("Content-Length") or 0)
        body = self.rfile.read(length).decode("utf-8", "replace")
        if length <= 0 or length > MAX_FORM_BODY:
            return self._send_json({"code": "bad_body"}, 400)
        form = urllib.parse.parse_qs(body, keep_blank_values=True)
        ssid = (form.get("ssid", [""])[0]).strip()
        password = form.get("password", [""])[0]
        if not ssid:
            return self._send_json({"code": "ssid_required"}, 400)
        if len(ssid.encode("utf-8")) >= 32 or (password and not 8 <= len(password.encode("utf-8")) < 64):
            return self._send_json({"code": "ap_invalid_arg"}, 400)

        state["ap_ssid"] = ssid
        state["ap_auth"] = bool(password)
        self._send_json({"ok": True})
        print(f"[mock] 保存配网热点: ssid={ssid!r} password={'***' if password else '(空=开放热点)'}")

    def _handle_mode(self):
        # 与固件 set_work_mode 一致：切 ap 会断开 WiFi 并开热点，立即生效
        length = int(self.headers.get("Content-Length") or 0)
        body = self.rfile.read(length).decode("utf-8", "replace")
        if length <= 0 or length > MAX_FORM_BODY:
            return self._send_json({"code": "bad_body"}, 400)
        mode = urllib.parse.parse_qs(body, keep_blank_values=True).get("mode", [""])[0].lower()
        if mode not in ("sta", "ap"):
            return self._send_json({"code": "bad_mode"}, 400)

        state["work_mode"] = mode
        if mode == "ap":
            state["connected"] = False
            state["ip"] = ""
            state["ap_active"] = True
        # 切回 sta 时不动 ap_active：真实固件里热点要等 STA 连上（GOT_IP 事件）
        # 才关，刚切过去还没连上时热点仍然开着
        self._send_json({"ok": True})
        print(f"[mock] 工作模式切到: {mode}")

    def log_message(self, fmt, *args):
        # 静默访问日志：页面 5 秒轮询两个接口，打了会刷屏
        pass


if __name__ == "__main__":
    server = ThreadingHTTPServer(("127.0.0.1", PORT), Handler)
    print(f"mock 服务已启动：http://127.0.0.1:{PORT} （Ctrl-C 退出）")
    server.serve_forever()
