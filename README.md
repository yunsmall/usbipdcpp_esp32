# ESP32 USB/IP Server

[![ESP-IDF](https://img.shields.io/badge/ESP--IDF-5.5-blue)](https://github.com/espressif/esp-idf)
[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](LICENSE)

An USB/IP server implementation for ESP32-S3 and ESP32-P4 based on [usbipdcpp](https://github.com/yunsmall/usbipdcpp) library. This project serves as a practical example of using usbipdcpp on ESP32 platforms.

> **Note**: USB device compatibility depends on the ESP32 chip's USB PHY speed support:
> - **ESP32-S3**: Supports **Full Speed** (12Mbps) and **Low Speed** (1.5Mbps) devices
> - **ESP32-P4**: Supports **High Speed** (480Mbps), **Full Speed** (12Mbps) and **Low Speed** (1.5Mbps) devices
>
> High Speed devices (480Mbps) may not work correctly on ESP32-S3 due to PHY limitations.

English | [中文](README-zh.md)

## ✨ Features

- 🔄 **Transparent USB Forwarding** - Export local USB devices to remote machines via USB/IP protocol
- 🔌 **Hot-plug Support** - Automatic device detection, enumeration, and cleanup on removal
- 🌐 **Multi-device Support** - USB hubs supported, multiple devices can be exported simultaneously
- ⚡ **Zero-Copy Performance** - Direct DMA buffer access eliminates data copying overhead, achieving optimal throughput
- 🛡️ **Robust Connection Handling** - Automatic cleanup when devices are unplugged during active sessions
- 🖥️ **Runtime WiFi Configuration** - Change WiFi over a web UI or a dedicated serial console; credentials persist in NVS across reboots — no recompilation needed
- 📊 **Device Status Panel** - Web UI and serial console list attached devices (busid, VID:PID, remote-client usage)

## 📋 Requirements

### Hardware
- ESP32-S3 or ESP32-P4 development board (USB OTG supported)
- USB devices (keyboards, mice, mass storage, etc.)
- USB hub (optional, for multiple devices)

> **One Board, Two Roles**: Flash this firmware onto your ESP32-S3 dev board and it becomes a USB/IP dongle; flash something else and it's back to being a regular dev board. No hardware modifications needed — just swap the firmware whenever you switch projects.

> **USB OTG Power**: Common ESP32-S3 DevKitC boards and their compatible counterparts have a USB OTG power solder pad on the back. Bridging this pad allows the board to supply power to connected USB devices. If you prefer not to modify the hardware, you can use a self-powered USB hub (one with an external power supply) instead — the hub will power the devices while the ESP32 handles data.

> **Speed Compatibility**: Ensure your ESP32 chip's USB PHY supports the USB speed type of your device. For example, High Speed UVC webcams require ESP32-P4.

> **Flash Size**: Default configuration assumes ESP32-S3 (8MB flash) and ESP32-P4 (32MB flash). Modify via `idf.py menuconfig` → `Serial flasher config` → `Flash size` if needed.

### Software
- ESP-IDF v5.5
- Linux machine with `usbip` tools installed (client side)

## 🚀 Quick Start

### 1. Clone the Repository

```bash
git clone --recursive https://github.com/yunsmall/usbipdcpp_esp32.git
cd usbipdcpp_esp32
```

### 2. Configure WiFi (Optional)

The checked-in per-target configs (`sdkconfig.defaults.esp32s3` / `esp32p4`) intentionally contain **no** WiFi credentials — a fresh build boots without network. Provide credentials one of two ways:

**Option A — compile-time default (connects on first boot):**

```bash
idf.py menuconfig
```

Navigate to `Usbipdcpp WiFi Configuration` and set `Usbipd WiFi SSID` / `Usbipd WiFi Password`.

**Option B — configure at runtime after flashing** via the web UI or the dedicated serial console (no recompile needed, credentials persist in NVS across reboots). See the *Management & Configuration* section below.

### 3. Build and Flash

```bash
idf.py build flash monitor
```

For specific chip targets, use the appropriate sdkconfig defaults:

```bash
# For ESP32-S3
idf.py -DSDKCONFIG_DEFAULTS="sdkconfig.defaults.esp32s3" build flash monitor

# For ESP32-P4
idf.py -DSDKCONFIG_DEFAULTS="sdkconfig.defaults.esp32p4" build flash monitor
```

> If the default sdkconfig doesn't take effect, explicitly specify it with `-DSDKCONFIG_DEFAULTS`.

### 4. Connect from Linux Client

On your Linux machine:

```bash
# Load USB/IP kernel modules
sudo modprobe vhci-hcd

# List available devices
sudo usbip list -r <ESP32_IP>

# Attach to a device
sudo usbip attach -r <ESP32_IP> -b <BUSID>
```

## 🖥️ Management & Configuration

WiFi and device status are manageable at runtime. WiFi credentials are stored in NVS (namespace `wifi`) and applied automatically on every boot — recompiling to switch networks is never required.

### Web UI (when online)

Open `http://<ESP32_IP>/` in a browser (HTTP port 80):

- **USB devices card** — attached devices with `busid`, `VID:PID` and usage state (idle / in use by a remote client). Auto-refreshes every 5 seconds.
- **WiFi card** (collapsed by default) — current SSID/IP, change credentials (the device disconnects and reconnects to the new AP), plus a wiring hint for the fallback serial console. Empty password = open network.

REST API: `GET /api/status` (connection state + config-port GPIOs), `GET /api/devices` (device list), `POST /api/wifi` (form-urlencoded `ssid=..&password=..`).

### Serial Console (when offline / misconfigured WiFi)

If the network is unreachable, configure through the **dedicated config UART** — the pins differ per chip (Kconfig: `USBIPD_CFG_UART_TX_GPIO` / `USBIPD_CFG_UART_RX_GPIO`):

| Chip       | Config UART TX | Config UART RX |
|------------|----------------|----------------|
| ESP32-S3   | GPIO17         | GPIO18         |
| ESP32-P4   | GPIO4          | GPIO5          |

Wire a USB-UART adapter **crossed**: adapter RX → device TX, adapter TX → device RX, GND common. Open the terminal at 115200 8N1 and use the built-in commands (`help` lists all):

| Command | Purpose |
|---------|---------|
| `wifi_set <ssid> [password]` | Set WiFi and reconnect (saved to NVS; omit password for an open network) |
| `wifi_show` / `wifi_reset` | Show current config / clear NVS back to compile-time defaults |
| `devices` | List attached USB devices (busid / VID:PID / usage state) |
| `mem` | Print heap usage |
| `logs` | Mirror the main UART0 log stream to the config port (Ctrl-C to stop) |
| `about` | What this firmware is and how to manage it |

The mirrored log stream is written with `\r\n` line endings so it renders correctly on real serial terminals.

### Local Web Preview (no flashing)

The web UI is a single static file (`main/web/index.html`) embedded at compile time. To iterate on layout/scripts without flashing the firmware: run `python main/web/mock_server.py` and open `http://127.0.0.1:8000`. The mock serves fake `/api/status` and `/api/devices` responses; edit the top of the script to preview different states (disconnected, disabled config port, long SSID…).

## 🏗️ Architecture

```
┌─────────────────────────────────────────────────────────┐
│                    Linux Client                          │
│  ┌─────────────┐    ┌─────────────┐    ┌─────────────┐  │
│  │   App A     │    │   App B     │    │   App C     │  │
│  └──────┬──────┘    └──────┬──────┘    └──────┬──────┘  │
│         └──────────────────┼──────────────────┘         │
│                            │                            │
│                    ┌───────┴───────┐                    │
│                    │   usbip-vhci  │                    │
│                    └───────┬───────┘                    │
└────────────────────────────┼────────────────────────────┘
                             │ TCP/IP Network
┌────────────────────────────┼────────────────────────────┐
│                    ┌───────┴───────┐                    │
│                    │ ESP32-S3      │                    │
│                    │ USB/IP Server │                    │
│                    └───────┬───────┘                    │
│                            │                            │
│         ┌──────────────────┼──────────────────┐         │
│         │                  │                  │         │
│    ┌────┴────┐       ┌─────┴─────┐     ┌─────┴─────┐   │
│    │ USB Hub │       │ USB Dev 1 │ ... │ USB Dev N │   │
│    └─────────┘       └───────────┘     └───────────┘   │
└─────────────────────────────────────────────────────────┘
```

## 📝 Tested Devices

| Device Type | Status | Notes |
|-------------|--------|-------|
| USB Keyboard | ✅ Working | |
| USB Mouse | ✅ Working | |
| USB Flash Drive (MSC) | ✅ Working | Bulk transfer tested |
| USB Audio | 🔄 Testing | |
| USB Webcam (UVC) | 🔄 Testing | Requires ESP32-P4 for High Speed |

> Bulk and interrupt transfers have been verified to work correctly. Ensure your ESP32 chip's USB PHY supports the target device's speed type.

## ⚡ Performance Optimization

This implementation leverages usbipdcpp's zero-copy architecture for maximum throughput:

- **Direct DMA Buffer Access**: USB transfer buffers are allocated in DMA-capable memory and accessed directly for network I/O, eliminating intermediate data copies
- **RAII Transfer Management**: `TransferHandle` automatically manages buffer lifecycle, ensuring proper cleanup without manual memory management
- **ESP32-Specific Optimizations**:
  - Bulk/Interrupt IN transfers aligned to endpoint Max Packet Size for hardware efficiency
  - Control transfer buffers pre-allocated with setup packet space
  - Object pooling for callback structures reduces allocation overhead

## 📚 Related Project

- [usbipdcpp](https://github.com/yunsmall/usbipdcpp) - A cross-platform USB/IP protocol library. This project is an ESP32 implementation using usbipdcpp.

## 📢 Commercial Use

If you use this project in a product, please display the following information prominently in your product documentation or about page:

- Project URL: `https://github.com/yunsmall/usbipdcpp_esp32`
- Author: `yunsmall` (GitHub)
- Contact: `yun_small@163.com`

## ⚠️ Known Issues

### Chunked Transfer — Currently Disabled

Large USB transfers (e.g. 65536 bytes for firmware flashing) require a single DMA-capable buffer of the same size. When DMA memory is fragmented, a large contiguous allocation can fail even though the total free space is sufficient.

**Chunked transfer** addresses this by splitting the large allocation into multiple smaller blocks (default: 16384 bytes each), trading one large contiguous allocation for several smaller ones — dramatically increasing the probability of successful allocation.

The chunked transfer implementation has been moved to the `feature/chunked-transfer` branch. To try it out, switch to that branch and build:

```bash
git checkout feature/chunked-transfer
idf.py build flash monitor
```

However, chunking is **disabled by default** (`enable_chunking = false` in `Esp32DeviceHandler.cpp`) due to a persistent timeout issue: when enabled, some bulk transfer scenarios (e.g. remote JLINK firmware flashing) cause the first chunk to NAK indefinitely while waiting for device data. The host times out after ~1 second, sends CMD_UNLINK, and the cycle repeats. The root cause has not been fully identified.

If you know how to fix this, pull requests are welcome.

### Hardware Recommendation

**ESP32-P4 is strongly recommended** over ESP32-S3 for USB/IP use. ESP32-S3's internal DMA-capable memory is very limited (~300KB), which can cause large USB transfer buffer allocations to fail under load. ESP32-P4 supports DMA access to PSRAM, which removes the need for chunking in most cases.

## 📄 License

Apache License 2.0