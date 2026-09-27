#include "WifiConnection.h"

#include <algorithm>
#include <chrono>
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
        // 记下原因供 apply_config 判定（密码错/找不到 AP 时提前结束等待）
        auto *event = static_cast<wifi_event_sta_disconnected_t *>(event_data);
        if (event != nullptr) {
            WifiConfigManager::instance().note_disconnect_reason(event->reason);
        }
        // 状态机：掉线回 Connecting 重新计时（换配置过程中的断开由 apply_config 收尾）
        WifiConfigManager::instance().note_sta_disconnected();
        // 置位并唤醒重连线程（重复置位无害：标志只表示"有请求待处理"）
        {
            std::lock_guard lock(self->reconnect_mutex_);
            self->reconnect_pending_ = true;
        }
        self->reconnect_cv_.notify_one();
    }
    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_CONNECTED) {
        // 成功连上 AP：退避清零，下次意外断线时从最短间隔（1s）重新开始
        self->backoff_exp_.store(0);
    }
    else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        auto *event = static_cast<ip_event_got_ip_t *>(event_data);
        if (event != nullptr) {
            ESP_LOGI(TAG, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
        }
        // 供 apply_config 判断"本次切换后已连上"（计数增长 = 拿到了新 IP）
        WifiConfigManager::instance().note_got_ip();
        // 状态机：连上 → Connected；配网热点的使命到此结束，该关掉回到纯 STA
        // （用户刚在热点里配好网，或旧 AP 自己恢复了，两种情况都该关）。
        // 这里只置请求：本回调跑在 esp_event 任务里，esp_wifi_set_mode 是与驱动
        // 交互的重量级调用，直接在此执行会阻塞其它 WiFi 事件的处理，交给看门狗线程
        WifiConfigManager::instance().note_sta_connected();
        self->ap_stop_requested_.store(true);
    }
}

esp_err_t WifiConnection::start()
{
    ESP_ERROR_CHECK(esp_netif_init());

    ESP_ERROR_CHECK(esp_event_loop_create_default());
    if (esp_netif_create_default_wifi_sta() == nullptr) {
        // 没有 netif 就没有 IP 可取（ip_str 按 WIFI_STA_DEF 查），
        // 连接判定和状态展示会一路空着，直接失败好过带病运行
        ESP_LOGE(TAG, "创建 STA netif 失败");
        return ESP_FAIL;
    }

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
    // 读 NVS 失败不中止：load_config 内部已把内存回退成编译期默认，而 NVS 损坏
    // 是持久故障——abort 只会让设备卡在重启循环，热点配网/网页/USB/IP 反而都没了
    if (wifi_manager.load_config() != ESP_OK) {
        ESP_LOGW(TAG, "WiFi 配置读取失败，已回退到编译期默认（详见上面的日志）");
    }
    const auto wifi_ssid = wifi_manager.ssid();
    const auto wifi_passwd = wifi_manager.password();

    wifi_config_t wifi_config{};
    // 只拷 size-1 字节，末位留给 {} 初始化出来的 0：strncpy 在 src 长度 >= n 时
    // 不写终止符，靠这一步保证拷完仍是合法 C 串
    strncpy(reinterpret_cast<char *>(wifi_config.sta.ssid), wifi_ssid.c_str(), std::size(wifi_config.sta.ssid)-1);
    strncpy(reinterpret_cast<char *>(wifi_config.sta.password), wifi_passwd.c_str(), std::size(wifi_config.sta.password)-1);
    // 截断了要说一声：Kconfig 的值不受 apply_config 的长度校验约束，静默截断会
    // 让"连不上但配的明明是对的"变成无头案
    if (wifi_ssid.size() > std::size(wifi_config.sta.ssid) - 1) {
        ESP_LOGW(TAG, "Kconfig 的 WiFi 名称超过 %u 字节，只按前 %u 字节连接",
                 static_cast<unsigned>(wifi_ssid.size()),
                 static_cast<unsigned>(std::size(wifi_config.sta.ssid) - 1));
    }
    if (wifi_passwd.size() > std::size(wifi_config.sta.password) - 1) {
        ESP_LOGW(TAG, "Kconfig 的 WiFi 密码超过 %u 字节，只按前 %u 字节连接",
                 static_cast<unsigned>(wifi_passwd.size()),
                 static_cast<unsigned>(std::size(wifi_config.sta.password) - 1));
    }
    wifi_config.sta.scan_method = WIFI_ALL_CHANNEL_SCAN;
    wifi_config.sta.sort_method = WIFI_CONNECT_AP_BY_SIGNAL;

    esp_pthread_cfg_t pthread_cfg = esp_pthread_get_default_config();
    pthread_cfg.prio = 10;
    pthread_cfg.pin_to_core = 1; // 设置核心1
    pthread_cfg.thread_name = "wifi_connect_thread";
    esp_pthread_set_cfg(&pthread_cfg);
    reconnect_thread_ = std::thread([this]() {
        while (!should_stop_.load()) {
            {
                std::unique_lock lock(reconnect_mutex_);
                reconnect_cv_.wait(lock, [this] {
                    return reconnect_pending_ || should_stop_.load();
                });
                if (should_stop_.load()) {
                    break;
                }
                reconnect_pending_ = false;
            }
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
    // esp_pthread_get_default_config() 是按 menuconfig 现场构造一份默认值
    // （IDF pthread/pthread.c 的实现，不读线程 TLS），拿到的是真默认，设回去即恢复。
    // 注意它恢复的是"出厂默认"而非"进来时的值"：调用方若先设过自定义配置会被这
    // 一对调用覆盖，当前唯一调用方 thread_main 进来时就是默认，满足这个前提
    esp_pthread_cfg_t default_cfg = esp_pthread_get_default_config();
    esp_pthread_set_cfg(&default_cfg);

    // 期望模式是 Ap 时这些 STA 动作也照做：这里只是把配置备好（没人调 connect，
    // 驱动不会自己连），而切回 STA 模式时 reconnect 正是拿这份配置去连——跳过的话
    // `wifi_mode sta` 要等到重启才连得上。热点本身由看门狗拉起：它排在
    // esp_wifi_start 之后，Ap 模式下 should_start_provisioning_ap 恒真
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    // 关掉 modem sleep（默认 WIFI_PS_MIN_MODEM）：省电模式下 AP 按 802.11
    // 省电协议把下行帧（含 TCP ACK）缓存到 beacon 窗口才投递，TCP 吞吐被压
    // 到 beacon 周期量级、延迟带抖动——usbip 是持续双向流量，两个方向都吃
    // 亏且设备常驻供电，省电的收益（几 mA）远小于稳定性损失
    // 不用 ESP_ERROR_CHECK：这是性能调优项，失败只该降级（吞吐变差），不该拿它
    // 决定设备能不能启动
    const esp_err_t ps_err = esp_wifi_set_ps(WIFI_PS_NONE);
    if (ps_err != ESP_OK) {
        ESP_LOGW(TAG, "关闭 modem sleep 失败（%s），吞吐可能下降", esp_err_to_name(ps_err));
    }

    // 配网看门狗：连续连不上（WifiConfigManager::AP_FALLBACK_SECONDS）就拉起配网
    // 热点，让手边没有 USB-TTL 的人也能配网；STA 一旦连上由 GOT_IP 事件置请求、
    // 本线程执行关闭。每秒轮询一次就够——超时是秒级的事，多等一秒无感。
    // 放在 esp_wifi_start 之后：热点操作要 WiFi 驱动起来才有意义，否则第一轮
    // 就在未启动的驱动上 set_mode（失败、刷错误日志，下一轮才自愈）
    {
        // 本线程要调 esp_wifi_set_mode/set_config（驱动接口在调用者栈上执行）
        // 并做日志格式化，默认 3KB 栈偏紧，撑到 4KB
        esp_pthread_cfg_t watchdog_cfg = esp_pthread_get_default_config();
        watchdog_cfg.stack_size = 4096;
        watchdog_cfg.thread_name = "ap_watchdog";
        esp_pthread_set_cfg(&watchdog_cfg);
    }
    provisioning_watchdog_thread_ = std::thread([this]() {
        while (!should_stop_.load()) {
            std::this_thread::sleep_for(std::chrono::seconds(1));
            if (should_stop_.load()) {
                break;
            }
            // 先处理关热点请求（STA 已连上，见 GOT_IP 事件）：只有真关掉了才清标志，
            // 关失败（set_mode 瞬时错误）时留着下一轮重试——清了就没人再提这件事，
            // STA 明明已连上、热点却一直广播；而成功时热点可能还没开（disable 直接
            // 返回 true），标志也得清掉，不然会误关下一次开的热点
            if (ap_stop_requested_.load() && disable_provisioning_ap()) {
                ap_stop_requested_.store(false);
            }
            if (!ap_active_.load() &&
                WifiConfigManager::instance().should_start_provisioning_ap()) {
                enable_provisioning_ap();
            }
        }
    });
    {
        // 同上：get_default_config 给的是 menuconfig 默认值，设回去即恢复
        esp_pthread_cfg_t watchdog_default_cfg = esp_pthread_get_default_config();
        esp_pthread_set_cfg(&watchdog_default_cfg);
    }

    ESP_LOGI(TAG, "wifi_init_sta finished.");
    return ESP_OK;
}

void WifiConnection::enable_provisioning_ap()
{
    if (ap_active_.load()) {
        return; // 已在运行（看门狗每秒查一次，这里幂等）
    }

    auto &manager = WifiConfigManager::instance();
    const std::string ap_ssid = manager.ap_ssid();
    const std::string ap_password = manager.ap_password();
    if (ap_ssid.empty()) {
        // 看门狗每秒来一次，而"Kconfig 与 NVS 都没配名称"是永远失败的配置错误：
        // 只在第一次报错，否则日志被每秒一条刷屏
        if (!ap_error_logged_.exchange(true)) {
            ESP_LOGE(TAG, "配网热点 SSID 为空（Kconfig 与 NVS 都没配），无法启动热点");
        }
        return;
    }

    // AP netif 懒创建：IDF 会一并拉起 DHCP server（默认网段 192.168.4.0/24），
    // 客户端连上热点即可访问 http://192.168.4.1/ 的现有配置页面
    if (ap_netif_ == nullptr) {
        ap_netif_ = esp_netif_create_default_wifi_ap();
        if (ap_netif_ == nullptr) {
            // 看门狗每秒重试：和下面 set_mode/set_config 失败一样只报第一次
            if (!ap_error_logged_.exchange(true)) {
                ESP_LOGE(TAG, "创建 AP netif 失败（同类失败不再重复报）");
            }
            return;
        }
    }

    wifi_config_t ap_config{};
    // 名称先截到驱动上限再拷，ssid_len 与实际拷入的字节数保持一致：Kconfig 里的
    // 名称/密码不受 apply_ap_config 的长度校验约束（那只管网页/串口改的路径），
    // 超长时若照抄 size() 会让驱动按超出实际内容的长度去读
    const std::size_t ap_ssid_len = std::min(ap_ssid.size(), sizeof(ap_config.ap.ssid) - 1);
    std::strncpy(reinterpret_cast<char *>(ap_config.ap.ssid), ap_ssid.c_str(), ap_ssid_len);
    ap_config.ap.ssid_len = static_cast<std::uint8_t>(ap_ssid_len);
    ap_config.ap.max_connection = 4;
    // 信道 1 只在"STA 未连上"时真正生效（热点就是这时候才开的）；APSTA 下 STA
    // 之后若连上别的信道的 AP，驱动会把热点拉到 STA 的信道，客户端收到 CSA 后
    // 跟随——这个窗口很短，STA 一连上就置 ap_stop_requested_ 把热点关掉
    ap_config.ap.channel = 1;
    if (ap_password.size() >= 8) {
        if (ap_password.size() > sizeof(ap_config.ap.password) - 1) {
            ESP_LOGW(TAG, "配网热点密码超过 %u 位（Kconfig），按前 %u 位设置",
                     static_cast<unsigned>(ap_password.size()),
                     static_cast<unsigned>(sizeof(ap_config.ap.password) - 1));
        }
        std::strncpy(reinterpret_cast<char *>(ap_config.ap.password), ap_password.c_str(),
                     sizeof(ap_config.ap.password) - 1);
        ap_config.ap.authmode = WIFI_AUTH_WPA2_PSK;
    }
    else {
        // 空密码 = 开放热点；不足 8 位的非空密码只可能来自 Kconfig（apply_ap_config
        // 保证经它写入的密码要么空、要么 ≥8 位），说一声，别让用户以为热点有密码
        if (!ap_password.empty()) {
            ESP_LOGW(TAG, "配网热点密码不足 8 位（Kconfig），将以开放热点启动");
        }
        ap_config.ap.authmode = WIFI_AUTH_OPEN;
    }

    // APSTA：STA 在后台继续重试（旧 AP 恢复的话能自己连回来），连上即关热点
    esp_err_t err = esp_wifi_set_mode(WIFI_MODE_APSTA);
    if (err != ESP_OK) {
        // 看门狗每秒重试：持续失败时同一行 ERROR 会刷屏，只报第一次
        if (!ap_error_logged_.exchange(true)) {
            ESP_LOGE(TAG, "切换到 APSTA 模式失败（同类失败不再重复报）: %s", esp_err_to_name(err));
        }
        return;
    }
    err = esp_wifi_set_config(WIFI_IF_AP, &ap_config);
    if (err != ESP_OK) {
        if (!ap_error_logged_.exchange(true)) {
            ESP_LOGE(TAG, "设置热点配置失败（同类失败不再重复报）: %s", esp_err_to_name(err));
        }
        esp_wifi_set_mode(WIFI_MODE_STA); // 回退，别把设备留在半吊子状态
        return;
    }

    // 开成功了：复位"只报一次"的标记，下次再失败时还能重新看到错误
    ap_error_logged_.store(false);
    ap_active_.store(true);
    manager.note_ap_started();
    if (ap_config.ap.authmode == WIFI_AUTH_OPEN) {
        ESP_LOGI(TAG, "配网热点已启动: SSID=%s（开放网络），连上后浏览器打开 http://192.168.4.1/ 配网",
                 ap_ssid.c_str());
    }
    else {
        // 密码位数按实际写进驱动的长度报（Kconfig 超长时上面已截断并告警）：
        // 用户拿旧密码连不上时，一眼能看出生效的不是他配的那串
        ESP_LOGI(TAG, "配网热点已启动: SSID=%s（WPA2，密码 %u 位），连上后浏览器打开 http://192.168.4.1/ 配网",
                 ap_ssid.c_str(),
                 static_cast<unsigned>(std::strlen(reinterpret_cast<const char *>(ap_config.ap.password))));
    }
}

bool WifiConnection::disable_provisioning_ap()
{
    if (!ap_active_.load()) {
        return true; // 本来就没开，算已关
    }
    // 回到纯 STA：热点关闭（AP netif 保留，下次进入配网模式直接复用）
    const esp_err_t err = esp_wifi_set_mode(WIFI_MODE_STA);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "关闭配网热点失败（保留请求，下一轮重试）: %s", esp_err_to_name(err));
        return false;
    }
    ap_active_.store(false);
    ESP_LOGI(TAG, "STA 已连上，配网热点已关闭");
    return true;
}

bool WifiConnection::is_ap_active() const
{
    return ap_active_.load();
}
