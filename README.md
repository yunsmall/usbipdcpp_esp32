# ESP32 USB/IP Server

[![ESP-IDF](https://img.shields.io/badge/ESP--IDF-5.5-blue)](https://github.com/espressif/esp-idf)
[![License](https://img.shields.io/badge/License-MIT-green.svg)](LICENSE)

An USB/IP server implementation for ESP32-S3 based on [usbipdcpp](https://github.com/yunsmall/usbipdcpp) library. This project serves as a practical example of using usbipdcpp on ESP32 platforms.

> **Note**: ESP32-S3 USB OTG only supports **Full Speed** (12Mbps) and **Low Speed** (1.5Mbps) devices. High Speed (480Mbps) devices may not work correctly.

English | [中文](README-zh.md)

## ✨ Features

- 🔄 **Transparent USB Forwarding** - Export local USB devices to remote machines via USB/IP protocol
- 🔌 **Hot-plug Support** - Automatic device detection, enumeration, and cleanup on removal
- 🌐 **Multi-device Support** - USB hubs supported, multiple devices can be exported simultaneously
- ⚡ **High Performance** - Optimized for bulk and interrupt transfers
- 🛡️ **Robust Connection Handling** - Automatic cleanup when devices are unplugged during active sessions

## 📋 Requirements

### Hardware
- ESP32-S3 development board (USB OTG supported)
- USB devices (keyboards, mice, mass storage, etc.)
- USB hub (optional, for multiple devices)

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

| Device Type | Status |
|-------------|--------|
| USB Keyboard | ✅ Working |
| USB Mouse | ✅ Working |
| USB Flash Drive (MSC) | 🔄 Testing |
| USB Audio | 🔄 Testing |
| USB Webcam (UVC) | 🔄 Testing |

## 📚 Related Project

- [usbipdcpp](https://github.com/yunsmall/usbipdcpp) - A cross-platform USB/IP protocol library. This project is an ESP32 implementation using usbipdcpp.

## 📄 License

MIT License