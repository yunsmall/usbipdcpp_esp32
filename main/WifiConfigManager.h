#pragma once

#include <chrono>
#include <condition_variable>
#include <initializer_list>
#include <mutex>
#include <string>
#include <utility>

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
     * @brief WiFi 工作状态（STA 连接与配网热点共用一个状态机）
     */
    enum class WifiState
    {
        Connecting, ///< STA 正在连接（含断线重连），配网热点未开
        Connected,  ///< STA 已连上（拿到 IP）
        Switching,  ///< 正在应用新的 STA 配置（apply_config 等结果中）
        /// 配网热点已启动（STA 是否在重试由 WifiWorkMode 决定）。与
        /// WifiConnection::is_ap_active() 说的是同一件事——那份是驱动事实，
        /// 本类不反向依赖 WifiConnection，才留了这个镜像；同步点只有
        /// note_ap_started（开）与 note_sta_connected（STA 连上后关）两处
        ApActive,
    };

    /**
     * @brief 期望的工作模式（存 NVS，重启后沿用；与上面的运行时状态不同——
     *        这是用户/配置表达的意图，设备按它决定要不要去连 WiFi）
     */
    enum class WifiWorkMode
    {
        Sta, ///< 连 WiFi 工作；连不上按 AP_FALLBACK_SECONDS 自动开配网热点，热点开启期间 STA 仍继续重试
        Ap,  ///< 只做配网热点，不主动连 WiFi、不重试（配网热点就是工作方式）
    };

    /**
     * @brief 连续连不上多久就启动配网热点（秒）：从进入 Connecting 起算，
     *        中途反复断线不会重置（见 set_state_locked）
     */
    static constexpr int AP_FALLBACK_SECONDS = 30;

    /**
     * @brief 读取 NVS 配置填充内部状态（无记录时回退编译期默认）
     *
     * 必须在 esp_wifi_init 之前调用一次（由启动流程负责）。返回值仅反映 NVS 能否
     * 打开——无记录、读取失败都回退编译期默认并返回 ESP_OK；打不开（分区损坏/
     * 满）时内存同样已回退编译期默认，返回错误只表示 NVS 不可用，调用方记日志
     * 继续启动即可，不必中止
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
     * @return ESP_OK 已连上并保存（NVS 写失败也算成功：连接已经生效，只是重启后
     *         回退旧配置，日志里有 ERROR）；ESP_ERR_TIMEOUT 超时未连上（内存与 STA
     *         配置均已回滚成旧值）；ESP_FAIL 被 AP 拒绝或找不到 AP（同上回滚）；
     *         ESP_ERR_INVALID_ARG 参数非法，什么都没动（空/全空白/含空字符/超长的
     *         ssid，含空字符、超长或不足 8 位的非空密码）；其它错误码 = STA 配置未能写入驱动
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
     * @brief 期望的工作模式（见 WifiWorkMode）
     */
    WifiWorkMode work_mode();

    /**
     * @brief 工作模式的字符串形式（"sta"/"ap"）：NVS 存的值、串口命令与网页
     *        的取值都用这一份，改名字只需要改这里
     */
    static const char *work_mode_name(WifiWorkMode mode);

    /**
     * @brief 解析工作模式字符串（大小写不敏感）；无法识别返回 false，不改 out
     */
    static bool parse_work_mode(std::string name, WifiWorkMode &out);

    /**
     * @brief 设置期望的工作模式并写 NVS，立即生效
     *
     * 与 apply_config 串行（同一把 apply 锁）：配网正在等连接结果时本调用排队，
     * 等它落定后再切——"切换配置"是个中间态，不该被另一个改模式的操作插进来。
     * 切到 Ap：断开当前 STA 连接（重连线程与事件回调都按模式让路，不会再连），
     * 配网热点由看门狗随即拉起；切到 Sta：立刻发起一次连接。
     * 模式没有变化时直接返回（不写 NVS——NVS 有擦写寿命）。NVS 写失败也算成功：
     * 模式与切换动作都已生效，只是重启后回退（日志里有 ERROR），与 apply_config 一致。
     */
    esp_err_t set_work_mode(WifiWorkMode mode);

    /**
     * @brief 该不该启动配网热点了：处于 Connecting 且已超过 AP_FALLBACK_SECONDS
     *
     * 供配网看门狗轮询（见 WifiConnection 的看门狗线程）。Switching 期间恒为
     * false——正在按用户新配置切换，结果由 apply_config 收尾
     */
    bool should_start_provisioning_ap();

    /**
     * @brief 事件回调用：STA 已连上（拿到 IP）→ Connected
     */
    void note_sta_connected();

    /**
     * @brief 事件回调用：STA 断开 → 回到 Connecting 重新计时（Switching 期间忽略：
     *        那是换配置的过程，状态由 apply_config 收尾）
     */
    void note_sta_disconnected();

    /**
     * @brief 配网热点已启动（看门狗调用）→ ApActive
     */
    void note_ap_started();

    /**
     * @brief 删除 NVS 里的 WiFi 记录（含配网热点配置；下次开机回退编译期默认），
     *        不改变当前连接
     */
    esp_err_t reset_config();

    /**
     * @brief 当前生效的配置（内部持锁返回拷贝，读与 apply 并发安全）
     */
    std::string ssid();
    std::string password();

    /**
     * @brief 配网热点的名称/密码（NVS 优先，无记录回退 Kconfig 的
     *        CONFIG_USBIPD_AP_SSID/PASSWORD；密码空串 = 开放热点）
     */
    std::string ap_ssid();
    std::string ap_password();

    /**
     * @brief 设置配网热点的名称/密码并写入 NVS
     *
     * 只改持久化配置，不动正在运行的热点——热点的名字/密码下次启动（或下次
     * 自动进入配网模式）才生效，避免把正连着热点配网的客户端踢下线。
     * ssid 为空或超长（>31）、password 非空但不足 8 位（WPA2 下限）或超长（>63）、
     * 两者任一含空字符返回 ESP_ERR_INVALID_ARG。
     */
    esp_err_t apply_ap_config(const std::string &ssid, const std::string &password);

    /**
     * @brief 持锁发起一次连接尝试，供重连线程调用（与 apply_config 的
     *        disconnect/set_config 互斥）
     */
    void reconnect();

    /**
     * @brief 是否已连上（关联到 AP 且已拿到 IP——只关联上但 DHCP 没成不算，
     *        否则状态与 IP 两处展示会自相矛盾）
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

    /**
     * @brief 状态迁移（须已持 mutex_）：只在状态真正变化时更新时间戳，
     *        重复进入同一状态不重置——重连期间会反复收到断开事件，
     *        每次都重置的话配网热点的超时永远等不到
     */
    void set_state_locked(WifiState state);

    /**
     * @brief 把工作模式写进 NVS（不含内存更新与切换动作；失败只记日志）
     */
    esp_err_t persist_work_mode(WifiWorkMode mode);

    /**
     * @brief 把若干 key/value 一次性写进 NVS 并提交（打开→写入→提交→关闭的
     *        样板收敛在这里）；任一写入失败即中止并返回该错误码
     */
    static esp_err_t nvs_write_all(std::initializer_list<std::pair<const char *, const char *>> items);

    /**
     * @brief 清空本命名空间的所有记录；命名空间不存在视为已清空（返回 ESP_OK）
     */
    static esp_err_t nvs_erase_namespace();

    /**
     * @brief 等本次切换的连接结果（须已持 apply_mutex_ 且 switch_sta_config 已完成）
     *
     * 连上的判据：切换后见过一次未关联、之后关联上的又是目标 SSID、且拿到 IP
     * （关联上但 DHCP 没成功同样不算）。密码错/AP 不在场这类确定性失败提前返回，
     * 不必等满 APPLY_TIMEOUT_SECONDS。
     * @param ssid 目标 AP 的 SSID
     * @param switch_started_at 发起切换的时刻（apply_config 在 switch_sta_config
     *        之前取）：超时截止与"失败宽限期"都从这一刻算起，不从进入等待算
     * @param rejected 出参：true = 被 AP 拒绝或找不到 AP，false = 超时
     * @return true = 已连上目标 AP
     */
    bool wait_connection_result(const std::string &ssid,
                                std::chrono::steady_clock::time_point switch_started_at,
                                bool &rejected);

    /**
     * @brief 切换失败后的回滚：STA 配置换回旧值重新连接、状态回退到切换前，
     *        内存与 NVS 都保持旧配置（须已持 apply_mutex_）
     */
    void rollback_after_failed_switch(const std::string &old_ssid, const std::string &old_password,
                                      WifiState state_before);

    static constexpr const char *NVS_NAMESPACE = "wifi";
    static constexpr const char *NVS_KEY_SSID = "ssid";
    static constexpr const char *NVS_KEY_PASSWORD = "passwd";
    static constexpr const char *NVS_KEY_AP_SSID = "ap_ssid";
    static constexpr const char *NVS_KEY_AP_PASSWORD = "ap_passwd";
    static constexpr const char *NVS_KEY_MODE = "mode";
    // NVS 里 mode 的取值（存字符串而非数字：翻 NVS 排障时一眼能看懂）
    static constexpr const char *NVS_MODE_STA = "sta";
    static constexpr const char *NVS_MODE_AP = "ap";

    std::mutex mutex_;
    std::string ssid_;
    std::string password_;
    std::string ap_ssid_;
    std::string ap_password_;
    WifiWorkMode work_mode_ = WifiWorkMode::Sta;

    // 状态机当前值与进入该状态的时刻（超时判定用，见 set_state_locked）
    WifiState state_ = WifiState::Connecting;
    std::chrono::steady_clock::time_point state_since_ = std::chrono::steady_clock::now();

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
