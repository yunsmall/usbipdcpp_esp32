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
- 🖥️ **WiFi 运行时配置** - 通过网页或专用串口控制台改 WiFi，凭据存 NVS、重启仍生效——无需重新编译
- 📊 **设备状态面板** - 网页与串口均可查看已接入设备（busid、VID:PID、远程占用状态）

## 📋 环境要求

### 硬件
- ESP32-S3 或 ESP32-P4 开发板（需支持 USB OTG）
- USB 设备（键盘、鼠标、U盘等）
- USB 集线器（可选，用于连接多个设备）

> **一板两用**：只要你有一块 ESP32-S3 开发板，刷入本程序即可变身 USB/IP 转发器；想开发其他项目时只需刷入新固件即可，无需任何硬件改动。一块板子既能当 USB 转发器又能当普通开发板，非常方便。
>
> **USB OTG 供电**：市面常见的 ESP32-S3 DevKitC 及其兼容开发板，背面一般有一个 USB OTG 供电焊点。将此焊点短接后，开发板即可为接入的 USB 设备供电。如果不想改动硬件，可以使用支持外接供电的 USB 集线器，由集线器为设备供电，同样可以正常使用。
>
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

### 2. 配置 WiFi（可选）

仓库中的按芯片配置文件（`sdkconfig.defaults.esp32s3` / `esp32p4`）有意**不含** WiFi 凭据——全新构建开机没有网络。二选一提供凭据：

**方式 A — 编译期默认（开机即连）：**

```bash
idf.py menuconfig
```

进入 `Usbipdcpp WiFi Configuration`，设置 `Usbipd WiFi SSID` / `Usbipd WiFi Password`。

**方式 B — 烧录后运行时配置**：通过网页或专用串口控制台（无需重编译，凭据存 NVS、重启仍生效）。见下方*管理与配置*章节。

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

## 🖥️ 管理与配置

WiFi 与设备状态都支持运行时管理。WiFi 凭据存 NVS（命名空间 `wifi`），每次开机自动应用——换网络永远不需要重新编译固件。

### 网页管理页（联网时）

浏览器打开 `http://<ESP32_IP>/`（HTTP 端口 80）：

- **USB 设备卡** — 当前接入的设备，显示 `busid`、`VID:PID` 与占用状态（空闲 / 被远程客户端使用）。每 5 秒自动刷新
- **WiFi 卡**（默认折叠）— 当前 SSID/IP；修改凭据（设备会断开当前连接重连到新 AP）；配错网时串口救急的接线提示。密码留空 = 开放网络

REST API：`GET /api/status`（连接状态 + 配置口 GPIO）、`GET /api/devices`（设备列表）、`POST /api/wifi`（form-urlencoded：`ssid=..&password=..`）

### 串口配置口（断网 / 配错 WiFi 时）

连不上网时，用**专用配置 UART** 配置——引脚随芯片不同（Kconfig：`USBIPD_CFG_UART_TX_GPIO` / `USBIPD_CFG_UART_RX_GPIO`）：

| 芯片 | 配置口 TX | 配置口 RX |
|------|-----------|-----------|
| ESP32-S3 | GPIO17 | GPIO18 |
| ESP32-P4 | GPIO4 | GPIO5 |

USB 转 TTL 串口线**交叉**接：适配器 RX → 设备 TX、适配器 TX → 设备 RX、GND 共地。终端 115200 8N1 打开后用内置命令（`help` 查看全部）：

| 命令 | 作用 |
|------|------|
| `wifi_set <ssid> [password]` | 设置 WiFi 并重连（存 NVS；省略密码 = 开放网络） |
| `wifi_show` / `wifi_reset` | 查看当前配置 / 清空 NVS 回编译期默认 |
| `devices` | 列出已接入 USB 设备（busid / VID:PID / 占用状态） |
| `mem` | 打印堆内存占用 |
| `logs` | 把 UART0 主日志镜像到配置口（Ctrl-C 停止） |
| `about` | 固件简介 |

镜像日志按 `\r\n` 行尾写出，真实串口终端上不会出现阶梯换行。

### 网页本地预览（不用烧写）

网页是编译期嵌入的单个静态文件（`main/web/index.html`）。不烧写直接调布局/脚本：运行 `python main/web/mock_server.py` 后浏览器打开 `http://127.0.0.1:8000`。mock 提供假的 `/api/status`、`/api/devices` 响应；改脚本顶部数据可预览不同状态（未连接、配置口未启用、超长 SSID…）。

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

## 🔌 在自己的项目中使用 usbipdcpp

USB/IP 服务器核心在 **`components/usbipdcpp`** 组件里（usbipdcpp submodule + 构建在 IDF USB-host 栈上的 `esp32_handler` 适配层），它**不依赖 `main/` 的任何东西**——`main/` 下的内容只是本固件的交互外壳（WiFi 管理、串口 console、网页、设备面板）。只想要转发引擎的话，把 `main/` 剥掉即可。

**把组件搬进你的项目：**

```bash
cp -r components/usbipdcpp <你的项目>/components/
# 组件内嵌 usbipdcpp git submodule，在你的仓库里补上：
git submodule add https://github.com/yunsmall/usbipdcpp <你的项目>/components/usbipdcpp/usbipdcpp
```

组件自己的 `idf_component_register` 已声明全部依赖（`asio spdlog usb pthread lwip sock_utils`），你的 `main` 只需要 `PRIV_REQUIRES usbipdcpp asio spdlog`。USB 设备走 IDF 标准 `usb` 组件（用集线器请开 `USB_HOST_HUBS_SUPPORTED`）。

**最小可用集成**——不带 WiFi/串口/网页；网络栈由你的应用负责，组件只监听 TCP：

```cpp
#include <thread>
#include <freertos/FreeRTOS.h>
#include <usb/usb_host.h>
#include "esp32_handler/Esp32Server.h"

// usb_host 库级事件循环：需要独立任务跑整个生命周期
static void usb_host_event_loop() {
    while (true) {
        uint32_t event_flags;
        ESP_ERROR_CHECK(usb_host_lib_handle_events(portMAX_DELAY, &event_flags));
    }
}

extern "C" void app_main() {
    // 主流程放进 std::thread：spdlog/asio 需要完整 pthread 环境
    // （原因见 main/esp32_usbipdcpp.cpp thread_main 上方注释）
    std::thread main_thread([&] {
        const usb_host_config_t host_cfg = {
                .skip_phy_setup = false,
                .intr_flags = ESP_INTR_FLAG_LEVEL3,
                .enum_filter_cb = nullptr,
        };
        ESP_ERROR_CHECK(usb_host_install(&host_cfg));
        std::thread(usb_host_event_loop).detach();

        usbipdcpp::Esp32Server server;
        server.init_client();   // 注册 usb_host client（设备热插拔）
        asio::ip::tcp::endpoint ep{asio::ip::tcp::v4(), 3240};
        auto ec = server.start(ep);   // 开始监听，内部自起网络/会话线程
        if (ec) { /* 处理监听失败 */ }
        while (true) {
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
    });
    main_thread.join();
}
```

`Esp32Server::start()` 内部自带网络与 client 事件线程；插上的 USB 设备自动枚举并导出——电脑端 `usbip list -r <esp32-ip>` 查看，`usbip attach -r <esp32-ip> -b <busid>` 连接。

注意：

- **网络层自己写**——组件不初始化 WiFi/以太网或任何 PHY；先自己写代码把网络拉起来（STA 连接、以太网、静态/DHCP 地址，按你的项目来），再在网络上启动服务器
- **组件范围**——usbipdcpp 只做三件事：注册 usb_host client（`Esp32Server::init_client`）、设备热插拔处理与绑定、TCP 上的 USB/IP 协议会话。其余全归你的应用，包括 `usb_host_install`（见上方示例）及其内部的 USB PHY 初始化
- `usb_host_install` 是进程级单例，必须在创建 `Esp32Server` 前恰好调用一次
- `Esp32Server::start` 不抛异常，失败通过返回值 `error_code` 报告；设备绑定失败内部记日志并回滚
- `main/` 下的文件是最佳用法参考——`esp32_usbipdcpp.cpp` 的 `thread_main` 展示了完整流程，WiFi 管理/串口/网页只是同一核心外围的可选交互

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

本项目利用 usbipdcpp 的零拷贝架构实现最大吞吐量：

- **直接 DMA 缓冲区访问**：USB 传输缓冲区分配在 DMA 可访问内存中，直接用于网络 I/O，消除中间数据拷贝
- **RAII 传输管理**：`TransferHandle` 自动管理缓冲区生命周期，无需手动内存管理
- **ESP32 专属优化**：
  - Bulk/Interrupt IN 传输按端点 Max Packet Size 对齐，提升硬件效率
  - 控制传输缓冲区预分配 setup packet 空间
  - 回调结构体对象池减少分配开销

## 📚 相关项目

- [usbipdcpp](https://github.com/yunsmall/usbipdcpp) - 跨平台 USB/IP 协议库。本项目是使用 usbipdcpp 在 ESP32 上的实现示例。

## 📢 商业使用

如果将本项目用于产品，请在产品介绍的醒目位置注明以下信息：

- 项目地址：`https://github.com/yunsmall/usbipdcpp_esp32`
- 作者：`yunsmall`（GitHub）
- 联系方式：`yun_small@163.com`

## ⚠️ 已知问题

### 分块传输 — 当前已禁用

大 USB 传输（如固件烧写的 65536 字节）需要一次性分配等大的 DMA 缓冲区。当 DMA 内存碎片化时，单次大块分配可能因找不到足够的连续空间而失败，尽管总空闲空间充足。

**分块传输**将大块分配拆成多个小块（默认 16384 字节），用几个小块分配替代一个大块分配，大幅提高分配成功率。

分块实现已移至 `feature/chunked-transfer` 分支。想体验分块传输的，请切换分支后编译：

```bash
git checkout feature/chunked-transfer
idf.py build flash monitor
```

但分块传输当前**默认禁用**（`Esp32DeviceHandler.cpp` 中 `enable_chunking = false`）。启用后，部分批量传输场景（如远程 JLINK 固件烧写）会反复超时——第一个分块因等待设备数据而 NAK，host 约 1 秒后超时发送 CMD_UNLINK，传输失败后 host 重试，循环往复。根因尚未完全定位。

欢迎提交 PR 修复。

### 硬件建议

**强烈推荐使用 ESP32-P4** 而非 ESP32-S3 用于 USB/IP。ESP32-S3 内部 DMA 可用内存极小（约 300KB），高负载下大 USB 传输缓冲区容易分配失败。ESP32-P4 支持 DMA 访问 PSRAM，大多数情况无需分块即可正常使用。

## 📄 许可证

Apache License 2.0