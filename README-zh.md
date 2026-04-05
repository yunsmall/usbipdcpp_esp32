# ESP32 USB/IP 服务器

[![ESP-IDF](https://img.shields.io/badge/ESP--IDF-5.5-blue)](https://github.com/espressif/esp-idf)
[![License](https://img.shields.io/badge/License-MIT-green.svg)](LICENSE)

基于 [usbipdcpp](https://github.com/yunsmall/usbipdcpp) 库的 ESP32-S3 USB/IP 服务器实现。本项目是 usbipdcpp 在 ESP32 平台上的使用示例。

> **注意**：ESP32-S3 USB OTG 仅支持 **Full Speed** (12Mbps) 和 **Low Speed** (1.5Mbps) 设备。High Speed (480Mbps) 设备可能无法正常工作。

[English](README.md) | 中文

## ✨ 功能特性

- 🔄 **透明 USB 转发** - 通过 USB/IP 协议将本地 USB 设备导出到远程机器
- 🔌 **热插拔支持** - 自动检测设备、枚举、拔出时自动清理
- 🌐 **多设备支持** - 支持 USB 集线器，可同时导出多个设备
- ⚡ **高性能传输** - 针对批量传输和中断传输优化
- 🛡️ **健壮的连接处理** - 设备在会话中被拔出时自动清理资源

## 📋 环境要求

### 硬件
- ESP32-S3 开发板（需支持 USB OTG）
- USB 设备（键盘、鼠标、U盘等）
- USB 集线器（可选，用于连接多个设备）

### 软件
- ESP-IDF v5.5
- 安装了 `usbip` 工具的 Linux 机器（客户端）

## 🚀 快速开始

### 1. 克隆仓库

```bash
git clone --recursive https://github.com/yunsmall/usbipdcpp_esp32.git
cd usbipdcpp_esp32
```

### 2. 配置 WiFi

通过 menuconfig 设置 WiFi 凭据：

```bash
idf.py menuconfig
```

进入 `Usbipdcpp WiFi Configuration`，设置：
- `Usbipd WiFi SSID` - WiFi 名称
- `Usbipd WiFi Password` - WiFi 密码

### 3. 编译烧录

```bash
idf.py build flash monitor
```

### 4. 从 Linux 客户端连接

在 Linux 机器上执行：

```bash
# 加载 USB/IP 内核模块
sudo modprobe vhci-hcd

# 查看可用设备
sudo usbip list -r <ESP32_IP>

# 连接设备
sudo usbip attach -r <ESP32_IP> -b <BUSID>
```

## 🏗️ 架构图

```
┌─────────────────────────────────────────────────────────┐
│                    Linux 客户端                          │
│  ┌─────────────┐    ┌─────────────┐    ┌─────────────┐  │
│  │   应用 A    │    │   应用 B    │    │   应用 C    │  │
│  └──────┬──────┘    └──────┬──────┘    └──────┬──────┘  │
│         └──────────────────┼──────────────────┘         │
│                            │                            │
│                    ┌───────┴───────┐                    │
│                    │   usbip-vhci  │                    │
│                    └───────┬───────┘                    │
└────────────────────────────┼────────────────────────────┘
                             │ TCP/IP 网络
┌────────────────────────────┼────────────────────────────┐
│                    ┌───────┴───────┐                    │
│                    │ ESP32-S3      │                    │
│                    │ USB/IP 服务器 │                    │
│                    └───────┬───────┘                    │
│                            │                            │
│         ┌──────────────────┼──────────────────┐         │
│         │                  │                  │         │
│    ┌────┴────┐       ┌─────┴─────┐     ┌─────┴─────┐   │
│    │USB 集线器│       │ USB 设备 1 │ ... │ USB 设备 N │   │
│    └─────────┘       └───────────┘     └───────────┘   │
└─────────────────────────────────────────────────────────┘
```

## 📝 已测试设备

| 设备类型 | 状态 |
|---------|------|
| USB 键盘 | ✅ 正常 |
| USB 鼠标 | ✅ 正常 |
| U 盘 (MSC) | 🔄 测试中 |
| USB 音频 | 🔄 测试中 |
| USB 摄像头 (UVC) | 🔄 测试中 |

## 📚 相关项目

- [usbipdcpp](https://github.com/yunsmall/usbipdcpp) - 跨平台 USB/IP 协议库。本项目是使用 usbipdcpp 在 ESP32 上的实现示例。

## 📄 许可证

MIT License