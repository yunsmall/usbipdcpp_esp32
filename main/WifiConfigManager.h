#pragma once

#include <condition_variable>
#include <mutex>
#include <string>

#include <esp_err.h>

/**
 * @brief WiFi 配置的存储与应用
 *
 * 配置持久化在 NVS（namespace "wifi"，key ssid/passwd，明文——ESP32 无加密存储的前提下
 * 接受）。启动时 load_config 优先取 NVS，无记录则回退编译期默认（Kconfig 的
 * CONFIG_USBIPD_WIFI_SSID/PASSWORD，首启即用、向后兼容）。
 *
 * 线程模型：apply_config 与 reconnect 持同一把内部锁，保证"set_config 换配置"与
 * "重连线程 connect"不会交错（重连线程只调 reconnect，connect 统一收敛到这里，
 * 避免两处直接调 esp_wifi_connect 造成的双 connect 竞态）。
 *
 * 应用配置的语义（apply_config）：阻塞等待连接结果（最长 APPLY_TIMEOUT_SECONDS 秒），
 * 连上（拿到 IP）才写 NVS 并更新内存；超时或密码错/AP 不在场则回滚到旧配置、
 * 不落 NVS——"设置失败"等于什么都没发生，重启后仍是旧配置。配错的凭据因此不会
 * 覆盖掉 NVS 里那份能用的旧配置。
 *
 * 单例：instance() 返回静态实例。
 */
class WifiConfigManager
{
public:
    static WifiConfigManager &instance();

    WifiConfigManager(const WifiConfigManager &) = delete;
    WifiConfigManager &operator=(const WifiConfigManager &) = delete;

    /**
     * @brief 读取 NVS 配置填充内部状态（无记录时回退编译期默认）
     *
     * 必须在 esp_wifi_init 之前调用一次（由启动流程负责），返回值仅反映 NVS 读写
     * 是否成功——NVS 无记录不是错误，正常回退默认值返回 ESP_OK
     */
    esp_err_t load_config();

    /**
     * @brief 应用新配置：断开当前连接 → 设置并连接新 AP → 等结果 → 连上才写 NVS
     *
     * 阻塞等待连接结果（最长 APPLY_TIMEOUT_SECONDS 秒）。连上（本次切换后拿到 IP）
     * 才把配置写入 NVS 并更新内存；超时，或出现密码错/找不到 AP 这类确定性失败，
     * 则把 STA 配置回滚成旧值重新连接并返回错误——失败的配置不进 NVS，重启后仍是
     * 旧配置。ssid 为空或超长（>31）、password 超长（>63）返回 ESP_ERR_INVALID_ARG，
     * 不触碰 NVS 与 WiFi。
     * @param ssid 目标 AP 的 SSID
     * @param password AP 密码；空串视为开放 AP
     * @return ESP_OK 已连上并保存；ESP_ERR_TIMEOUT 超时未连上；ESP_FAIL 被 AP 拒绝
     *         或找不到 AP；ESP_ERR_INVALID_ARG 参数非法；其它为 set_config/NVS 级错误
     */
    esp_err_t apply_config(const std::string &ssid, const std::string &password);

    /**
     * @brief apply_config 等待连接结果的上限（秒），console/网页据此向用户措辞
     */
    static constexpr int APPLY_TIMEOUT_SECONDS = 15;

    /**
     * @brief WiFi 事件回调用：记录一次断开及其原因，并唤醒正在等结果的 apply_config
     *
     * apply_config 据此把"密码错/找不到 AP"这类确定性失败提前判掉，不必等满超时。
     * 事件回调跑在 esp_event 任务上下文（非 ISR），在等待锁内更新
     */
    void note_disconnect_reason(int reason);

    /**
     * @brief WiFi 事件回调用：有 IP 事件到达时唤醒等待中的 apply_config
     *
     * 是否真的连上目标 AP 由 apply_config 按"当前关联的 SSID 就是目标 + 已拿到 IP"
     * 判定（见其等待循环），这里只负责唤醒，不记录状态
     */
    void note_got_ip();

    /**
     * @brief 删除 NVS 里的 WiFi 记录（下次开机回退编译期默认），不改变当前连接
     */
    esp_err_t reset_config();

    /**
     * @brief 当前生效的配置（内部持锁返回拷贝，读与 apply 并发安全）
     */
    std::string ssid();
    std::string password();

    /**
     * @brief 持锁发起一次连接尝试，供重连线程调用（与 apply_config 的
     *        disconnect/set_config 互斥）
     */
    void reconnect();

    /**
     * @brief 是否已关联到 AP（esp_wifi_sta_get_ap_info 成功即有 AP 信息）
     */
    bool is_connected();

    /**
     * @brief 当前 STA 的 IPv4 地址字符串；未获取到 IP 返回空串
     */
    std::string ip_str();

private:
    WifiConfigManager() = default;

    /**
     * @brief 断开当前连接、把 STA 配置设为给定值并发起连接
     *
     * apply_config 换配置与它失败后的回滚共用。内部持配置锁，与重连线程互斥；
     * ssid 为空时只断开不重连（回滚到"从未配置过"的状态）。调用方须已持有
     * apply_mutex_（见 apply_config 的串行化说明）。
     * @return set_config 失败时返回其错误码（此时 STA 配置保持原值）
     */
    esp_err_t switch_sta_config(const std::string &ssid, const std::string &password);

    /**
     * @brief 当前 STA 是否已拿到可用的 IPv4 地址（0.0.0.0 视为还没拿到）
     */
    bool has_ip();

    static constexpr const char *NVS_NAMESPACE = "wifi";
    static constexpr const char *NVS_KEY_SSID = "ssid";
    static constexpr const char *NVS_KEY_PASSWORD = "passwd";

    std::mutex mutex_;
    std::string ssid_;
    std::string password_;
    bool loaded_ = false;

    // 串行化 apply：等待连接结果最长十几秒，网页与串口同时改时第二个调用排队执行，
    // 避免两个等待互相看到对方的连接事件而误判
    std::mutex apply_mutex_;

    // 最近一次断开原因与唤醒等待的条件变量：事件回调（note_*）在锁内更新并
    // notify，apply_config 在锁内检查条件后进入等待——检查与等待原子化，
    // 两者之间到达的事件不会丢失（见两个 note_ 方法注释）
    std::mutex wait_mutex_;
    std::condition_variable wait_cv_;
    int last_disconnect_reason_ = 0;
};
