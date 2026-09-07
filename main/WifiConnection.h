#pragma once

#include <atomic>
#include <cstdint>
#include <semaphore>
#include <thread>

#include <esp_err.h>
#include <esp_event.h>   // esp_event_base_t（事件回调签名用）

/**
 * @brief STA 模式 WiFi 连接生命周期管理：初始化、连接、断线自动重连
 *
 * 从 esp32_usbipdcpp.cpp 拆出（该文件只留启动编排）：netif/esp_wifi 初始化、
 * 事件回调与重连线程收敛到本类。配置的存储与应用仍归 WifiConfigManager——
 * 本类只负责"连接"动作本身，且 reconnect 全部经 WifiConfigManager 内部锁，
 * 与 apply_config 的换配置互斥（线程模型详见 WifiConfigManager.h）。
 *
 * 单例：instance() 返回静态实例。
 */
class WifiConnection
{
public:
    static WifiConnection &instance();

    WifiConnection(const WifiConnection &) = delete;
    WifiConnection &operator=(const WifiConnection &) = delete;

    /**
     * @brief 初始化 netif/event loop/esp_wifi，注册事件回调，按已存配置连接，
     *        并启动断线自动重连线程（esp_wifi 初始化用 ESP_ERROR_CHECK，失败
     *        直接终止——WiFi 是主功能，起不来没有继续运行的意义）
     */
    esp_err_t start();

private:
    WifiConnection() = default;

    // esp_event 回调（注册时 arg 传 this）：STA_START 触发首次连接，
    // STA_DISCONNECTED 释放重连信号量，GOT_IP 打印地址
    static void event_handler(void *arg, esp_event_base_t event_base,
                              int32_t event_id, void *event_data);

    // 断线重连信号：STA_DISCONNECTED release，重连线程 acquire 后按退避补连
    std::binary_semaphore reconnect_semaphore_{0};
    // 重连退避指数（0~6 → 1~64 秒），成功连上 AP（STA_CONNECTED）清零。
    // esp_event 回调任务与重连线程并发读写它，必须原子
    std::atomic<std::uint32_t> backoff_exp_{0};
    std::atomic_bool should_stop_{false};
    std::thread reconnect_thread_;

    static const char *TAG;
};
