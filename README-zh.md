# ESP32 USB/IP 服务器

[![ESP-IDF](https://img.shields.io/badge/ESP--IDF-5.5-blue)](https://github.com/espressif/esp-idf)
[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](LICENSE)

基于 [usbipdcpp](https://github.com/yunsmall/usbipdcpp) 库的 ESP32-S3 和 ESP32-P4 USB/IP 服务器实现。本项目是 usbipdcpp 在 ESP32 平台上的使用示例。

> **注意**：USB 设备兼容性取决于 ESP32 芯片的 USB PHY 速度支持：
> - **ESP32-S3**：支持 **Full Speed** (12Mbps) 和 **Low Speed** (1.5Mbps) 设备
> - **ESP32-P4**：支持 **High Speed** (480Mbps)、**Full Speed** (12Mbps) 和 **Low Speed** (1.5Mbps) 设备
>
> 由于 PHY 限制，High Speed 设备 (480Mbps) 在 ESP32-S3 上可能无法正常工作。

[English](README.md) | 中文

## ✨ 功能特性

- 🔄 **透明 USB 转发** - 通过 USB/IP 协议将本地 USB 设备导出到远程机器
- 🔌 **热插拔支持** - 自动检测设备、枚举、拔出时自动清理
- 🌐 **多设备支持** - 支持 USB 集线器，可同时导出多个设备
- ⚡ **零拷贝高性能** - 直接访问 DMA 缓冲区，消除数据拷贝开销，实现极致吞吐量
- 🛡️ **健壮的连接处理** - 设备在会话中被拔出时自动清理资源

## 📋 环境要求

### 硬件
- ESP32-S3 或 ESP32-P4 开发板（需支持 USB OTG）
- USB 设备（键盘、鼠标、U盘等）
- USB 集线器（可选，用于连接多个设备）

> **速度兼容性**：请确保 ESP32 芯片的 USB PHY 支持目标设备的 USB 速度类型。例如，High Speed 的 UVC 摄像头需要 ESP32-P4。

> **Flash 大小**：默认配置为 ESP32-S3 (8MB flash) 和 ESP32-P4 (32MB flash)。如需修改，请通过 `idf.py menuconfig` → `Serial flasher config` → `Flash size` 更改。

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

针对不同芯片目标，使用对应的 sdkconfig 默认配置：

```bash
# ESP32-S3
idf.py -DSDKCONFIG_DEFAULTS="sdkconfig.defaults.esp32s3" build flash monitor

# ESP32-P4
idf.py -DSDKCONFIG_DEFAULTS="sdkconfig.defaults.esp32p4" build flash monitor
```

> 如果默认 sdkconfig 未生效，请使用 `-DSDKCONFIG_DEFAULTS` 显式指定。

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

| 设备类型 | 状态 | 备注 |
|---------|------|------|
| USB 键盘 | ✅ 正常 | |
| USB 鼠标 | ✅ 正常 | |
| U 盘 (MSC) | ✅ 正常 | Bulk 传输已测试 |
| USB 音频 | 🔄 测试中 | |
| USB 摄像头 (UVC) | 🔄 测试中 | High Speed 需要 ESP32-P4 |

> Bulk 和中断传输已验证可正常工作。请确保 ESP32 芯片的 USB PHY 支持目标设备的速度类型。

## ⚡ 性能优化

本项目利用 usbipdcpp v1.0.1 的零拷贝架构实现最大吞吐量：

- **直接 DMA 缓冲区访问**：USB 传输缓冲区分配在 DMA 可访问内存中，直接用于网络 I/O，消除中间数据拷贝
- **RAII 传输管理**：`TransferHandle` 自动管理缓冲区生命周期，无需手动内存管理
- **ESP32 专属优化**：
  - Bulk/Interrupt IN 传输按端点 Max Packet Size 对齐，提升硬件效率
  - 控制传输缓冲区预分配 setup packet 空间
  - 回调结构体对象池减少分配开销

## 📚 相关项目

- [usbipdcpp](https://github.com/yunsmall/usbipdcpp) - 跨平台 USB/IP 协议库。本项目是使用 usbipdcpp 在 ESP32 上的实现示例。

## 📄 许可证

Apache License 2.0