#include "WifiConfigManager.h"

#include <chrono>
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

// 密码错 / AP 不在场这类断开原因：凭据或 SSID 有问题，重试也是同样结局。
// apply_config 见到就立刻判失败，不用等满超时（IDF 的事件 reason 取值见 esp_wifi_types.h）
bool is_definite_failure(int reason)
{
    switch (reason) {
        case WIFI_REASON_NO_AP_FOUND:
        case WIFI_REASON_AUTH_FAIL:
        case WIFI_REASON_4WAY_HANDSHAKE_TIMEOUT:
        case WIFI_REASON_HANDSHAKE_TIMEOUT:
            return true;
        default:
            return false;
    }
}

// 切换配置后这段时间内的"确定性失败"不采信（见 apply_config 的等待循环）：
// 新配置的连接失败最早也要几百毫秒（全信道扫描 + 认证）才可能出现，更早出现的
// 失败只可能是旧连接迟到的失败事件刚好被处理到了
constexpr auto kFailureGracePeriod = std::chrono::milliseconds(500);

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

esp_err_t WifiConfigManager::switch_sta_config(const std::string &ssid, const std::string &password)
{
    std::lock_guard lock(mutex_);

    // 清零放在断开旧连接之前：旧连接此前记录下的失败原因不能算到新配置头上。
    // 断开动作本身报的是"主动离开"，不在确定性失败集合里，不会误判
    {
        std::lock_guard wait_lock(wait_mutex_);
        last_disconnect_reason_ = 0;
    }
    // 断开旧连接（触发 DISCONNECTED 事件 → 重连线程 reconnect，与新连接互斥；
    // 若从未连接成功 disconnect 会返回错误，忽略即可）
    esp_wifi_disconnect();

    if (ssid.empty()) {
        // 空 SSID（仅回滚到"从未配置过"时出现）没有可连的目标，只断开、不 set_config
        return ESP_OK;
    }

    wifi_config_t wifi_config{};
    std::strncpy(reinterpret_cast<char *>(wifi_config.sta.ssid), ssid.c_str(),
                 sizeof(wifi_config.sta.ssid) - 1);
    if (!password.empty()) {
        std::strncpy(reinterpret_cast<char *>(wifi_config.sta.password), password.c_str(),
                     sizeof(wifi_config.sta.password) - 1);
    }
    // 扫描方式与现有启动流程一致（全信道扫描、按信号选 AP）
    wifi_config.sta.scan_method = WIFI_ALL_CHANNEL_SCAN;
    wifi_config.sta.sort_method = WIFI_CONNECT_AP_BY_SIGNAL;

    esp_err_t err = esp_wifi_set_config(WIFI_IF_STA, &wifi_config);
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

esp_err_t WifiConfigManager::apply_config(const std::string &ssid, const std::string &password)
{
    if (ssid.empty() || ssid.size() >= MAX_SSID_LEN || password.size() >= MAX_PASSPHRASE_LEN) {
        ESP_LOGE(TAG, "非法配置: ssid 长度 %zu（1~%u），password 长度 %zu（0~%u）",
                 ssid.size(), static_cast<unsigned>(MAX_SSID_LEN) - 1,
                 password.size(), static_cast<unsigned>(MAX_PASSPHRASE_LEN) - 1);
        return ESP_ERR_INVALID_ARG;
    }

    // 串行化：等连接结果最长十几秒，期间第二个 apply（网页与串口同时改）排队执行
    std::lock_guard apply_lock(apply_mutex_);

    std::string old_ssid;
    std::string old_password;
    {
        std::lock_guard lock(mutex_);
        old_ssid = ssid_;
        old_password = password_;
    }

    esp_err_t err = switch_sta_config(ssid, password);
    if (err != ESP_OK) {
        // 未写 NVS、内存未变；STA 配置保持原值（重连线程继续用旧配置）
        return err;
    }

    // 等结果：连上以"切换后见过一次未关联，之后关联上的又是目标 SSID，且拿到 IP"
    // 为准（关联上但 DHCP 失败同样不能用）。两个坑都排在判据里：
    // - 只数 GOT_IP 次数不够——旧连接若在切换瞬间刚好连上（重连线程正在重试时会
    //   这样），也会产生一次 GOT_IP，会把还没连上的新配置误判成成功写进 NVS；
    // - disconnect 是异步的，生效前 esp_wifi_sta_get_ap_info 还报告着旧 AP——
    //   新旧 SSID 相同（只改密码）时，残留的旧连接会被当成新配置已连上。
    // 密码错/找不到 AP 这类确定性失败重试也是同样结局，提前退出；其余事件等满
    // 上限（DHCP 慢的 AP 拿到 IP 要几秒）
    //
    // 事件驱动：WiFi 事件回调（note_*）在结果出现时唤醒本线程，不轮询。条件检查
    // 在等待锁内做——事件回调更新状态也要拿同一把锁，于是"检查未满足 → 进入等待"
    // 与"事件到达"之间不会交错，不会漏掉唤醒（否则要干等到超时）
    bool connected = false;
    bool rejected = false;
    {
        // 锁只圈住等待本身：回滚路径的 switch_sta_config 也要拿这把锁（非递归锁，
        // 同一线程不能重入），后面的 NVS 写入等耗时操作也不该压着事件回调
        std::unique_lock wait_lock(wait_mutex_);
        const auto switched_at = std::chrono::steady_clock::now();
        const auto deadline = switched_at + std::chrono::seconds(APPLY_TIMEOUT_SECONDS);
        bool saw_disconnected = false;
        while (true) {
            wifi_ap_record_t ap_info{};
            const bool associated = esp_wifi_sta_get_ap_info(&ap_info) == ESP_OK;
            if (!associated) {
                saw_disconnected = true;
            }
            connected = saw_disconnected && associated &&
                        std::strncmp(reinterpret_cast<const char *>(ap_info.ssid),
                                     ssid.c_str(), sizeof(ap_info.ssid)) == 0 &&
                        has_ip();
            // 确定性失败要过了宽限期才采信：新配置的失败最早也要几百毫秒
            // （全信道扫描 + 认证）才可能出现，更早的只能是旧连接迟到的失败
            // 事件被处理到了，不能算到新配置头上
            rejected = !connected && saw_disconnected &&
                       is_definite_failure(last_disconnect_reason_) &&
                       std::chrono::steady_clock::now() - switched_at >= kFailureGracePeriod;
            if (connected || rejected) {
                break;
            }
            if (wait_cv_.wait_until(wait_lock, deadline) == std::cv_status::timeout) {
                // 等满上限还没等到结果（唤醒可能是虚假的，循环会再查一次条件）
                break;
            }
        }
    }

    if (!connected) {
        ESP_LOGW(TAG, "新配置未连上（%s），回滚原配置", rejected ? "AP 拒绝或不在场" : "超时");
        // 回滚：不落 NVS、内存保持旧值、STA 配置也换回旧值，否则重连线程会一直
        // 拿刚设进去的（连不上的）新配置重试
        switch_sta_config(old_ssid, old_password);
        return rejected ? ESP_FAIL : ESP_ERR_TIMEOUT;
    }

    // 连上才持久化。NVS 写失败只记日志不报错：网络已经切过去了，返回失败反而
    // 与设备当前状态不符（重启后会回退旧配置，日志里有说明）
    nvs_handle_t handle;
    err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &handle);
    if (err == ESP_OK) {
        err = nvs_set_str(handle, NVS_KEY_SSID, ssid.c_str());
        if (err == ESP_OK) {
            err = nvs_set_str(handle, NVS_KEY_PASSWORD, password.c_str());
        }
        if (err == ESP_OK) {
            err = nvs_commit(handle);
        }
        nvs_close(handle);
    }
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "NVS 写入失败（连接已生效，但重启后会回退旧配置）: %s", esp_err_to_name(err));
    }

    // 内存状态最后更新：等待期间并发读到的仍是旧配置，与"新连接还没成功"的实际
    // 状态一致（成功后两者一起切换）
    {
        std::lock_guard lock(mutex_);
        ssid_ = ssid;
        password_ = password;
    }
    ESP_LOGI(TAG, "已连上新 AP 并保存配置");
    return ESP_OK;
}

void WifiConfigManager::note_disconnect_reason(int reason)
{
    // 更新在等待锁内、唤醒在锁外：与 apply_config 的"检查条件 → 进入等待"互斥，
    // 结果不会落在两者之间的窗口里被漏掉
    {
        std::lock_guard wait_lock(wait_mutex_);
        last_disconnect_reason_ = reason;
    }
    wait_cv_.notify_all();
}

void WifiConfigManager::note_got_ip()
{
    // 只负责唤醒：是否真的连上了目标 AP 由等待侧按"关联 SSID + 有 IP"判定，
    // 这里不记录状态（见 apply_config 的等待循环）
    wait_cv_.notify_all();
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

bool WifiConfigManager::has_ip()
{
    // "0.0.0.0" 是尚未获取到地址时的占位，不能算有 IP
    const std::string ip = ip_str();
    return !ip.empty() && ip != "0.0.0.0";
}
