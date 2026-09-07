#pragma once

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
     * @brief 应用新配置：写 NVS → 更新状态 → 断开当前连接 → 设置新配置 → 发起连接
     *
     * 当前连接会被断开并重连到新 AP（无网时直接发起连接）。内部持锁，
     * 与 reconnect 互斥。ssid 为空或超长（>31）、password 超长（>63）返回
     * ESP_ERR_INVALID_ARG，不触碰 NVS 与 WiFi。
     * @param ssid 目标 AP 的 SSID
     * @param password AP 密码；空串视为开放 AP
     */
    esp_err_t apply_config(const std::string &ssid, const std::string &password);

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

    static constexpr const char *NVS_NAMESPACE = "wifi";
    static constexpr const char *NVS_KEY_SSID = "ssid";
    static constexpr const char *NVS_KEY_PASSWORD = "passwd";

    std::mutex mutex_;
    std::string ssid_;
    std::string password_;
    bool loaded_ = false;
};
