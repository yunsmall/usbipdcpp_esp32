#include "WifiConfigManager.h"

#include <cstring>

#include <esp_log.h>
#include <esp_netif.h>
#include <esp_wifi.h>
#include <nvs.h>
#include <nvs_flash.h>

#include "sdkconfig.h"

namespace
{
constexpr const char *TAG = "WifiConfigManager";

// 长度上限直接用 esp_wifi 提供的宏：MAX_SSID_LEN（32，含终止符）、
// MAX_PASSPHRASE_LEN（64，含终止符），避免自定义常量与宏撞名/不一致

// 从 NVS 读字符串：目标缓冲不足时返回 ESP_ERR_NVS_INVALID_LENGTH，
// 调用方按"无记录"处理并回退默认
esp_err_t nvs_read_string(nvs_handle_t handle, const char *key, std::string &out)
{
    std::size_t len = 0;
    esp_err_t err = nvs_get_str(handle, key, nullptr, &len);
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        return err;
    }
    if (err != ESP_OK) {
        return err;
    }
    // len 含终止符
    out.resize(len - 1);
    err = nvs_get_str(handle, key, out.data(), &len);
    if (err != ESP_OK) {
        out.clear();
    }
    return err;
}

} // anonymous namespace

WifiConfigManager &WifiConfigManager::instance()
{
    static WifiConfigManager manager;
    return manager;
}

esp_err_t WifiConfigManager::load_config()
{
    std::lock_guard lock(mutex_);
    esp_err_t err = ESP_OK;

    // 独立 open/close：配置极少读写，不留长生命周期句柄（NVS 句柄数有限，
    // 其它模块也可能使用 nvs_flash）
    nvs_handle_t handle;
    err = nvs_open(NVS_NAMESPACE, NVS_READONLY, &handle);
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        // 命名空间不存在 = 从未配置过
        ssid_ = CONFIG_USBIPD_WIFI_SSID;
        password_ = CONFIG_USBIPD_WIFI_PASSWORD;
        loaded_ = true;
        return ESP_OK;
    }
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "NVS open %s failed: %s", NVS_NAMESPACE, esp_err_to_name(err));
        return err;
    }

    std::string stored_ssid;
    std::string stored_password;
    err = nvs_read_string(handle, NVS_KEY_SSID, stored_ssid);
    if (err == ESP_OK) {
        ssid_ = stored_ssid;
        // passwd 缺失视为开放 AP（空密码）
        if (nvs_read_string(handle, NVS_KEY_PASSWORD, stored_password) == ESP_OK) {
            password_ = stored_password;
        }
        else {
            password_.clear();
        }
        ESP_LOGI(TAG, "从 NVS 读取 WiFi 配置");
    }
    else if (err == ESP_ERR_NVS_NOT_FOUND) {
        ssid_ = CONFIG_USBIPD_WIFI_SSID;
        password_ = CONFIG_USBIPD_WIFI_PASSWORD;
    }
    else {
        ESP_LOGE(TAG, "读取 NVS 配置失败: %s", esp_err_to_name(err));
    }

    nvs_close(handle);
    loaded_ = true;
    return ESP_OK;
}

esp_err_t WifiConfigManager::apply_config(const std::string &ssid, const std::string &password)
{
    if (ssid.empty() || ssid.size() >= MAX_SSID_LEN || password.size() >= MAX_PASSPHRASE_LEN) {
        ESP_LOGE(TAG, "非法配置: ssid 长度 %zu（1~%u），password 长度 %zu（0~%u）",
                 ssid.size(), static_cast<unsigned>(MAX_SSID_LEN) - 1,
                 password.size(), static_cast<unsigned>(MAX_PASSPHRASE_LEN) - 1);
        return ESP_ERR_INVALID_ARG;
    }

    // 先落 NVS 再应用：若 set_config/连接失败，重启后仍会用新配置重试，
    // 比"内存成功但 NVS 失败"的不一致状态更好排查
    nvs_handle_t handle;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "NVS open 失败: %s", esp_err_to_name(err));
        return err;
    }
    err = nvs_set_str(handle, NVS_KEY_SSID, ssid.c_str());
    if (err == ESP_OK) {
        err = nvs_set_str(handle, NVS_KEY_PASSWORD, password.c_str());
    }
    if (err == ESP_OK) {
        err = nvs_commit(handle);
    }
    nvs_close(handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "NVS 写入失败: %s", esp_err_to_name(err));
        return err;
    }
    ESP_LOGI(TAG, "新 WiFi 配置已写入 NVS");

    std::lock_guard lock(mutex_);
    ssid_ = ssid;
    password_ = password;

    // 断开旧连接（触发 DISCONNECTED 事件 → 重连线程 reconnect，与新连接互斥；
    // 若从未连接成功 disconnect 会返回错误，忽略即可）
    esp_wifi_disconnect();

    wifi_config_t wifi_config{};
    std::strncpy(reinterpret_cast<char *>(wifi_config.sta.ssid), ssid_.c_str(),
                 sizeof(wifi_config.sta.ssid) - 1);
    if (!password_.empty()) {
        std::strncpy(reinterpret_cast<char *>(wifi_config.sta.password), password_.c_str(),
                     sizeof(wifi_config.sta.password) - 1);
    }
    // 扫描方式与现有启动流程一致（全信道扫描、按信号选 AP）
    wifi_config.sta.scan_method = WIFI_ALL_CHANNEL_SCAN;
    wifi_config.sta.sort_method = WIFI_CONNECT_AP_BY_SIGNAL;

    err = esp_wifi_set_config(WIFI_IF_STA, &wifi_config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_wifi_set_config 失败: %s", esp_err_to_name(err));
        return err;
    }

    // 持锁发起连接：与重连线程的 reconnect 互斥，不会有双 connect
    // （DISCONNECTED 事件的 reconnect 在本函数返回后才可能执行）
    err = esp_wifi_connect();
    if (err != ESP_OK) {
        // 已处于连接中/未启动等，重连线程后续会补连，不打 ERROR 刷屏
        ESP_LOGI(TAG, "esp_wifi_connect 返回 %s（重连线程会继续尝试）", esp_err_to_name(err));
    }
    return ESP_OK;
}

esp_err_t WifiConfigManager::reset_config()
{
    nvs_handle_t handle;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &handle);
    if (err != ESP_OK) {
        // 命名空间不存在 = 本来就没配置过
        if (err == ESP_ERR_NVS_NOT_FOUND) {
            return ESP_OK;
        }
        ESP_LOGE(TAG, "NVS open 失败: %s", esp_err_to_name(err));
        return err;
    }
    err = nvs_erase_all(handle);
    if (err == ESP_OK) {
        err = nvs_commit(handle);
    }
    nvs_close(handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "清除 NVS 失败: %s", esp_err_to_name(err));
        return err;
    }

    // 回退编译期默认：内存里立即生效，下次开机 load_config 取默认值
    {
        std::lock_guard lock(mutex_);
        ssid_ = CONFIG_USBIPD_WIFI_SSID;
        password_ = CONFIG_USBIPD_WIFI_PASSWORD;
    }
    ESP_LOGI(TAG, "WiFi 配置已重置为编译期默认");
    return ESP_OK;
}

std::string WifiConfigManager::ssid()
{
    std::lock_guard lock(mutex_);
    return ssid_;
}

std::string WifiConfigManager::password()
{
    std::lock_guard lock(mutex_);
    return password_;
}

void WifiConfigManager::reconnect()
{
    std::lock_guard lock(mutex_);
    // 与 apply_config 的 set_config/connect 互斥：拿不到锁说明配置正在更换，
    // 本次重连放弃（apply_config 内部已发起新连接）
    esp_err_t err = esp_wifi_connect();
    if (err != ESP_OK) {
        ESP_LOGI(TAG, "esp_wifi_connect 返回 %s", esp_err_to_name(err));
    }
}

bool WifiConfigManager::is_connected()
{
    wifi_ap_record_t ap_info{};
    return esp_wifi_sta_get_ap_info(&ap_info) == ESP_OK;
}

std::string WifiConfigManager::ip_str()
{
    esp_netif_t *netif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
    if (netif == nullptr) {
        return {};
    }
    esp_netif_ip_info_t ip_info{};
    if (esp_netif_get_ip_info(netif, &ip_info) != ESP_OK) {
        return {};
    }
    char buf[16] = {};
    esp_ip4addr_ntoa(&ip_info.ip, buf, sizeof(buf));
    return buf;
}
