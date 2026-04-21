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

## 📋 Requirements

### Hardware
- ESP32-S3 or ESP32-P4 development board (USB OTG supported)
- USB devices (keyboards, mice, mass storage, etc.)
- USB hub (optional, for multiple devices)

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

### 2. Configure WiFi

Set your WiFi credentials via menuconfig:

```bash
idf.py menuconfig
```

Navigate to `Usbipdcpp WiFi Configuration` and set:
- `Usbipd WiFi SSID`
- `Usbipd WiFi Password`

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

This implementation leverages usbipdcpp v1.0.1's zero-copy architecture for maximum throughput:

- **Direct DMA Buffer Access**: USB transfer buffers are allocated in DMA-capable memory and accessed directly for network I/O, eliminating intermediate data copies
- **RAII Transfer Management**: `TransferHandle` automatically manages buffer lifecycle, ensuring proper cleanup without manual memory management
- **ESP32-Specific Optimizations**:
  - Bulk/Interrupt IN transfers aligned to endpoint Max Packet Size for hardware efficiency
  - Control transfer buffers pre-allocated with setup packet space
  - Object pooling for callback structures reduces allocation overhead

## 📚 Related Project

- [usbipdcpp](https://github.com/yunsmall/usbipdcpp) - A cross-platform USB/IP protocol library. This project is an ESP32 implementation using usbipdcpp.

## 📄 License

Apache License 2.0