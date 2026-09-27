#pragma once

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <mutex>
#include <thread>

#include <esp_err.h>
#include <esp_event.h>   // esp_event_base_t（事件回调签名用）
#include <esp_netif.h>   // esp_netif_t（配网热点的 AP 接口）

/**
 * @brief STA 模式 WiFi 连接生命周期管理：初始化、连接、断线自动重连、配网热点
 *
 * 从 esp32_usbipdcpp.cpp 拆出（该文件只留启动编排）：netif/esp_wifi 初始化、
 * 事件回调、重连线程与配网看门狗收敛到本类。配置的存储与应用仍归
 * WifiConfigManager——本类只负责"连接/开热点"这些动作本身，且 reconnect 全部
 * 经 WifiConfigManager 内部锁，与 apply_config 的换配置互斥（线程模型详见
 * WifiConfigManager.h）。
 *
 * 配网热点：STA 连续连不上（WifiConfigManager::AP_FALLBACK_SECONDS）时由看门狗
 * 拉起（APSTA，STA 继续后台重试），用户连上热点用浏览器配网，STA 一旦连上由
 * GOT_IP 事件关掉热点。
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
     *        并启动断线自动重连线程与配网看门狗（esp_wifi 初始化用 ESP_ERROR_CHECK，
     *        失败直接终止——WiFi 是主功能，起不来没有继续运行的意义）
     */
    esp_err_t start();

    /**
     * @brief 启动配网热点（名称/密码取自 WifiConfigManager，NVS 优先回退 Kconfig）
     *
     * 首次调用创建 AP netif（IDF 自带 DHCP server，客户端连上即可访问
     * http://192.168.4.1/，现有 HTTP 配置服务直接复用）。已在运行则无副作用。
     */
    void enable_provisioning_ap();

    /**
     * @brief 关闭配网热点回到纯 STA（STA 已连上时调用；AP netif 保留供复用）
     * @return true = 现在的状态就是热点已关（含原本就没开）；false = 本次关闭
     *         失败，调用方应保留关热点请求、下一轮重试
     */
    bool disable_provisioning_ap();

    /**
     * @brief 配网热点当前是否在运行
     */
    bool is_ap_active() const;

private:
    WifiConnection() = default;

    // esp_event 回调（注册时 arg 传 this）：STA_START 触发首次连接，
    // STA_DISCONNECTED 置重连请求，GOT_IP 打印地址
    static void event_handler(void *arg, esp_event_base_t event_base,
                              int32_t event_id, void *event_data);

    // 断线重连请求：STA_DISCONNECTED 置位并唤醒，重连线程消费后按退避补连。
    // 用 mutex + condition_variable 而不是信号量：信号量的 release() 有"计数
    // 不得超过 max()"的前置条件（标准未定义，各实现行为不同），而掉线抖动时
    // 断开事件会连着来、消费端却在退避几秒到几十秒，计数很容易顶满；一个 bool
    // 标志天然吸收突发，重复请求合并掉不影响行为（多连一次是幂等的）
    std::mutex reconnect_mutex_;
    std::condition_variable reconnect_cv_;
    bool reconnect_pending_ = false;
    // 重连退避指数（0~6 → 1~64 秒），成功连上 AP（STA_CONNECTED）清零。
    // esp_event 回调任务与重连线程并发读写它，必须原子
    std::atomic<std::uint32_t> backoff_exp_{0};
    // 单例生命周期即进程（无析构/重启路径），当前没有任何地方置位——留着是给
    // 将来加退出逻辑用：置 true 并 join 下面两个线程即可让循环退出
    std::atomic_bool should_stop_{false};
    std::thread reconnect_thread_;

    // 配网热点：AP netif 懒创建（首次启用热点时），DHCP server 由 IDF 内部拉起。
    // 只有看门狗线程读写（enable_provisioning_ap 里），所以不用 atomic：关热点
    // 不销毁 netif，下次开热点直接复用，别的线程也不碰它
    esp_netif_t *ap_netif_ = nullptr;
    std::atomic_bool ap_active_{false};
    // 关热点请求：GOT_IP 事件回调只置位，真正的关闭在看门狗线程里做——
    // esp_wifi_set_mode 是与驱动交互的重量级调用，在 esp_event 任务里直接调
    // 会阻塞其它 WiFi 事件的处理
    std::atomic_bool ap_stop_requested_{false};
    // 开热点失败的错误日志只报一次（热点名为空、set_mode/set_config 失败都算）：
    // 看门狗每秒重试，持续失败会让同一行 ERROR 每秒刷屏；开成功后复位，
    // 下次再失败时还能重新看到
    std::atomic_bool ap_error_logged_{false};
    // 配网看门狗线程：每秒查一次"是否该拉热点了"（见 WifiConfigManager::
    // should_start_provisioning_ap），ESP32 单射频，热点只在真连不上时才开
    std::thread provisioning_watchdog_thread_;

    static const char *TAG;
};
