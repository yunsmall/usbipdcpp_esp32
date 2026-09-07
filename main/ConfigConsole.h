#pragma once

#include <cstdarg>

#include <esp_err.h>
#include <esp_log.h>

namespace usbipdcpp
{
    class Esp32Server;
}

/**
 * @brief 配置用独立 UART 上的 esp_console REPL 与系统日志镜像
 *
 * 背景：系统日志（spdlog/ESP_LOG）走默认 console UART（USB 串口，UART0），设备日常只
 * 插 USB 口供电无人看日志。配网时用 CH340 两根线接独立 UART（Kconfig 可配 GPIO），
 * 此 UART 上跑 REPL 提供 wifi set/show/reset、devices、mem、logs 等命令，两条数据流
 * 物理隔离。
 *
 * 日志镜像（logs 命令）把 UART0 正在输出的日志流同时写到配置口：
 * - spdlog：向 default logger 的 sinks 里追加一次 UartMirrorSink。sinks() 是裸引用
 *   vector，运行中增删与并发打日志是 data race，因此 sink 只在 init（启动早期）加
 *   一次，运行中只翻转原子开关，sink 内部检查开关决定是否写配置口
 * - ESP_LOG：esp_log_set_vprintf 接管，先调原实现（UART0 照常）再按开关镜像
 */
class ConfigConsole
{
public:
    static ConfigConsole &instance();

    ConfigConsole(const ConfigConsole &) = delete;
    ConfigConsole &operator=(const ConfigConsole &) = delete;

    /**
     * @brief 启动 REPL 并注册命令（内部先装日志镜像钩子，必须在其他任务开始大量
     *        打日志之前调用）。不阻塞：REPL 跑在独立任务
     */
    esp_err_t init();

    /**
     * @brief 日志镜像到配置口开关（logs 命令调用，原子开关）
     */
    static void set_log_mirror(bool enabled);

    /**
     * @brief 挂接 usbip 服务器实例供 devices 命令查询设备状态。由启动流程在
     *        server.start 成功后调用（与 HttpConfigApi 共用同一实例，见
     *        esp32_usbipdcpp.cpp）；未挂接时 devices 命令提示服务未启动
     */
    void set_server(usbipdcpp::Esp32Server *server);

    /**
     * @brief 已挂接的 usbip 服务器（nullptr = 尚未挂接）
     */
    usbipdcpp::Esp32Server *server() const;

private:
    ConfigConsole() = default;

    // esp_log 的自定义 vprintf：UART0 原样输出 + 镜像开启时写到配置口
    static int mirror_log_vprintf(const char *fmt, va_list args);

    // 由 esp_log_set_vprintf 返回的原 vprintf（保持 UART0 输出）
    static vprintf_like_t s_orig_log_vprintf;

    // 挂接的 usbip 服务器（devices 命令的数据源，见 set_server 注释）
    usbipdcpp::Esp32Server *esp32_server_ = nullptr;
};
