#include "WifiConnection.h"

#include <cstring>
#include <iterator>

#include <esp_event.h>
#include <esp_log.h>
#include <esp_netif.h>
#include <esp_pthread.h>
#include <esp_wifi.h>

#include "WifiConfigManager.h"
#include "sdkconfig.h"

namespace
{
// 退避上限指数：2^6 = 64 秒（重连间隔封顶）
constexpr std::uint32_t MAX_BACKOFF_EXP = 6;
} // anonymous namespace

const char *WifiConnection::TAG = "WifiConnection";

WifiConnection &WifiConnection::instance()
{
    static WifiConnection connection;
    return connection;
}

void WifiConnection::event_handler(void *arg, esp_event_base_t event_base,
                                   int32_t event_id, void *event_data)
{
    auto *self = static_cast<WifiConnection *>(arg);
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        // 走配置锁统一发起连接（与 apply_config 的换配置互斥，见 WifiConfigManager）
        WifiConfigManager::instance().reconnect();
    }
    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        ESP_LOGI(TAG, "connect to the AP fail");
        self->reconnect_semaphore_.release();
    }
    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_CONNECTED) {
        // 成功连上 AP：退避清零，下次意外断线时从最短间隔（1s）重新开始
        self->backoff_exp_.store(0);
    }
    else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        auto *event = static_cast<ip_event_got_ip_t *>(event_data);
        ESP_LOGI(TAG, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
    }
}

esp_err_t WifiConnection::start()
{
    ESP_ERROR_CHECK(esp_netif_init());

    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
        ESP_EVENT_ANY_ID,
        &event_handler,
        this,
        &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
        IP_EVENT_STA_GOT_IP,
        &event_handler,
        this,
        &instance_got_ip));

    // WiFi 配置来源：NVS 有记录用 NVS，否则编译期默认（见 WifiConfigManager）
    auto &wifi_manager = WifiConfigManager::instance();
    ESP_ERROR_CHECK(wifi_manager.load_config());
    const auto wifi_ssid = wifi_manager.ssid();
    const auto wifi_passwd = wifi_manager.password();

    wifi_config_t wifi_config{};
    strncpy(reinterpret_cast<char *>(wifi_config.sta.ssid), wifi_ssid.c_str(), std::size(wifi_config.sta.ssid)-1);
    strncpy(reinterpret_cast<char *>(wifi_config.sta.password), wifi_passwd.c_str(), std::size(wifi_config.sta.password)-1);
    wifi_config.sta.scan_method = WIFI_ALL_CHANNEL_SCAN;
    wifi_config.sta.sort_method = WIFI_CONNECT_AP_BY_SIGNAL;

    esp_pthread_cfg_t pthread_cfg = esp_pthread_get_default_config();
    pthread_cfg.prio = 10;
    pthread_cfg.pin_to_core = 1; // 设置核心1
    pthread_cfg.thread_name = "wifi_connect_thread";
    esp_pthread_set_cfg(&pthread_cfg);
    reconnect_thread_ = std::thread([this]() {
        while (!should_stop_.load()) {
            reconnect_semaphore_.acquire();
            if (should_stop_.load())
                break;
            // 指数退避重连：AP 不在场/密码错误等会连续失败，每次都立刻重试只会
            // 高频触发扫描刷日志耗电，失败越久间隔越长：1s → 2s → … → 64s 封顶。
            // 间隔翻倍在本次 connect 之前完成、清零在 STA_CONNECTED（必然晚于
            // connect 成功）之后发生，两条路径在时序上不会互相误判
            const std::uint32_t exp = backoff_exp_.load();
            if (exp < MAX_BACKOFF_EXP) {
                backoff_exp_.store(exp + 1);
            }
            std::this_thread::sleep_for(std::chrono::seconds(1u << exp));
            if (should_stop_.load())
                break;
            ESP_LOGI(TAG, "wifi reconnecting (backoff %us)", 1u << exp);
            // 持配置锁发起连接：与 apply_config 的换配置互斥（见 WifiConfigManager）
            WifiConfigManager::instance().reconnect();
        }
    });
    esp_pthread_cfg_t default_cfg = esp_pthread_get_default_config();
    esp_pthread_set_cfg(&default_cfg);

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    // 关掉 modem sleep（默认 WIFI_PS_MIN_MODEM）：省电模式下 AP 按 802.11
    // 省电协议把下行帧（含 TCP ACK）缓存到 beacon 窗口才投递，TCP 吞吐被压
    // 到 beacon 周期量级、延迟带抖动——usbip 是持续双向流量，两个方向都吃
    // 亏且设备常驻供电，省电的收益（几 mA）远小于稳定性损失
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));

    ESP_LOGI(TAG, "wifi_init_sta finished.");
    return ESP_OK;
}
