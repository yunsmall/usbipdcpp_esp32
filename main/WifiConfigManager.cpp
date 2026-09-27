#include "WifiConfigManager.h"

#include <algorithm>
#include <cctype>

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
    if (err != ESP_OK) {
        return err; // 含 ESP_ERR_NVS_NOT_FOUND（无记录）
    }
    if (len == 0) {
        out.clear(); // 契约上至少含终止符，防御一下别让下面的减法下溢
        return ESP_OK;
    }
    // 按含终止符的长度分配、读完整段再缩掉：NVS 里存空串（开放热点密码就是）
    // 时 len=1，若先 resize(len-1)=0 再让 nvs_get_str 写终止符就越界了
    out.resize(len);
    err = nvs_get_str(handle, key, out.data(), &len);
    if (err != ESP_OK) {
        out.clear();
        return err;
    }
    // 成功时 len 是驱动写回的实际长度（含终止符），正常至少 1；真给 0 的话
    // len-1 会下溢成 SIZE_MAX 直接 abort，宁可按空串处理
    out.resize(len > 0 ? len - 1 : 0);
    return ESP_OK;
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

// 状态名（状态迁移日志用）
const char *state_name(WifiConfigManager::WifiState state)
{
    switch (state) {
        case WifiConfigManager::WifiState::Connecting:
            return "连接中";
        case WifiConfigManager::WifiState::Connected:
            return "已连接";
        case WifiConfigManager::WifiState::Switching:
            return "切换配置中";
        case WifiConfigManager::WifiState::ApActive:
            return "配网热点已启动";
    }
    return "未知";
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
        ap_ssid_ = CONFIG_USBIPD_AP_SSID;
        ap_password_ = CONFIG_USBIPD_AP_PASSWORD;
        work_mode_ = WifiWorkMode::Sta;
        return ESP_OK;
    }
    if (err != ESP_OK) {
        // 打不开（分区损坏/满）也回退编译期默认再返回错误：内存里若留着空配置，
        // 下面 WiFi 驱动就会拿到空 SSID。返回值让调用方知道 NVS 不可用，但不必
        // 中止启动——配网热点、网页配置、USB/IP 都不依赖 NVS
        ESP_LOGE(TAG, "NVS open %s failed: %s，回退编译期默认", NVS_NAMESPACE, esp_err_to_name(err));
        ssid_ = CONFIG_USBIPD_WIFI_SSID;
        password_ = CONFIG_USBIPD_WIFI_PASSWORD;
        ap_ssid_ = CONFIG_USBIPD_AP_SSID;
        ap_password_ = CONFIG_USBIPD_AP_PASSWORD;
        work_mode_ = WifiWorkMode::Sta;
        return err;
    }

    std::string stored_ssid;
    std::string stored_password;
    err = nvs_read_string(handle, NVS_KEY_SSID, stored_ssid);
    if (err == ESP_OK && !stored_ssid.empty()) {
        // passwd 条目不存在 = 配网时留空（开放 AP），这是正常配置；读失败（条目
        // 损坏等）不能也当成空密码——那会静默把设备改成用开放网络去连，日志里
        // 对"这次为什么连不上"毫无线索。读不全就整体回退编译期默认，SSID 与密码
        // 保持配套
        const esp_err_t pass_err = nvs_read_string(handle, NVS_KEY_PASSWORD, stored_password);
        if (pass_err == ESP_OK || pass_err == ESP_ERR_NVS_NOT_FOUND) {
            ssid_ = stored_ssid;
            password_ = pass_err == ESP_OK ? stored_password : std::string{};
            ESP_LOGI(TAG, "从 NVS 读取 WiFi 配置");
        }
        else {
            ESP_LOGE(TAG, "读取 NVS 密码失败: %s，SSID 与密码一起回退编译期默认", esp_err_to_name(pass_err));
            ssid_ = CONFIG_USBIPD_WIFI_SSID;
            password_ = CONFIG_USBIPD_WIFI_PASSWORD;
        }
    }
    else if (err == ESP_ERR_NVS_NOT_FOUND || err == ESP_OK) {
        // 无记录，或记录是空串（NVS 损坏/旧固件写进去的）：都按"没配过"回退编译期
        // 默认。空 SSID 不能带着走——它会一路传到 esp_wifi_set_config，那里会报参数
        // 错，而调用点是 ESP_ERROR_CHECK，等于把设备推进重启循环
        if (err == ESP_OK) {
            ESP_LOGW(TAG, "NVS 里的 SSID 是空串，视为未配置，回退编译期默认");
        }
        ssid_ = CONFIG_USBIPD_WIFI_SSID;
        password_ = CONFIG_USBIPD_WIFI_PASSWORD;
    }
    else {
        // 读失败（NVS 损坏等）：回退编译期默认，别把空 SSID 交给 WiFi 驱动——
        // 空目标的扫描/连接没有意义，不如拿已知的默认配置再试一次
        ESP_LOGE(TAG, "读取 NVS 配置失败: %s，回退编译期默认", esp_err_to_name(err));
        ssid_ = CONFIG_USBIPD_WIFI_SSID;
        password_ = CONFIG_USBIPD_WIFI_PASSWORD;
    }

    // 配网热点配置独立于 STA 配置：缺哪项回退哪项的编译期默认
    std::string stored_ap;
    if (nvs_read_string(handle, NVS_KEY_AP_SSID, stored_ap) == ESP_OK) {
        ap_ssid_ = stored_ap;
    }
    else {
        ap_ssid_ = CONFIG_USBIPD_AP_SSID;
    }
    if (nvs_read_string(handle, NVS_KEY_AP_PASSWORD, stored_ap) == ESP_OK) {
        ap_password_ = stored_ap;
    }
    else {
        ap_password_ = CONFIG_USBIPD_AP_PASSWORD;
    }

    // 期望模式：读不出来或不认识的值都按 STA（行为与加这个配置项之前一致）
    std::string stored_mode;
    if (nvs_read_string(handle, NVS_KEY_MODE, stored_mode) != ESP_OK ||
        !parse_work_mode(stored_mode, work_mode_)) {
        work_mode_ = WifiWorkMode::Sta;
    }

    nvs_close(handle);
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
    // 若从未连接成功 disconnect 会返回错误，忽略即可）。
    // 持 mutex_ 调驱动是安全的：esp_wifi_disconnect 只向 wifi 任务投递断开命令、
    // 不等事件处理完，而处理 DISCONNECTED 的回调（note_sta_disconnected）也要拿
    // 这把锁——那时本函数早已返回。哪天 IDF 改成同步等事件，这里就会死锁，
    // 届时要把它挪到锁外
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
    // 上限取 MAX_SSID_LEN-1：32 字节 SSID 虽然合规，但 STA 配置的 ssid[32] 没有
    // 终止符空间、长度判定要看驱动怎么处理（配网热点那边有显式 ssid_len 才敢填满），
    // 保守拒绝，代价是放不进去一个 32 字节的 SSID。
    // 全空白按无效处理：网页端已经 trim 过，这里兜住串口/脚本直接调的情况——
    // 放行的话要白等一次 15 秒连接超时（前后带空格但中间有内容的仍允许，
    // SSID 本来就是任意字节）。
    // 内嵌 NUL 也不能放行：esp_wifi 的 STA 配置是以 NUL 结尾的 char[32]，存进
    // NVS 的却是完整串，会出现"存的和驱动实际连的不是一个东西"
    const bool ssid_blank = ssid.find_first_not_of(" \t\r\n") == std::string::npos;
    const bool ssid_has_nul = ssid.find('\0') != std::string::npos;
    // 密码同理不能含 NUL：NVS 与驱动都按 C 字符串处理，"1234567\0X" 长度 9 看着
    // 够 8 位，实际存下来、连出去的只有前 7 位，连不上还查不出原因
    const bool password_has_nul = password.find('\0') != std::string::npos;
    // 非空密码同样要够 WPA2 的 8 位下限（空 = 开放网络）：1~7 位交给驱动只会在
    // 连接阶段失败，报出来的错会含糊成"密码错误或找不到 AP"
    const bool password_too_short = !password.empty() && password.size() < 8;
    if (ssid.empty() || ssid_blank || ssid_has_nul || ssid.size() >= MAX_SSID_LEN ||
        password.size() >= MAX_PASSPHRASE_LEN || password_has_nul || password_too_short) {
        ESP_LOGE(TAG, "非法配置: ssid 长度 %zu（1~%u，且不能全空白/含空字符），password 长度 %zu（0 或 8~%u，且不能含空字符）",
                 ssid.size(), static_cast<unsigned>(MAX_SSID_LEN) - 1,
                 password.size(), static_cast<unsigned>(MAX_PASSPHRASE_LEN) - 1);
        return ESP_ERR_INVALID_ARG;
    }

    // 串行化：等连接结果最长十几秒，期间第二个 apply（网页与串口同时改）排队执行
    std::lock_guard apply_lock(apply_mutex_);

    std::string old_ssid;
    std::string old_password;
    // 初值只是占位，紧接着就在锁内被真实状态覆盖；给个已知合法值是为了避开
    // "可能未初始化"的编译警告
    WifiState state_before = WifiState::Connecting;
    {
        std::lock_guard lock(mutex_);
        old_ssid = ssid_;
        old_password = password_;
        state_before = state_;
        // 进入切换状态：等待期间配网看门狗不判超时（它只看 Connecting），
        // 结果由本函数收尾（成功 → Connected，失败回滚 → Connecting 重新计时）
        set_state_locked(WifiState::Switching);
    }

    // 切换动作本身也要时间（disconnect/set_config/connect），宽限期与 15 秒超时
    // 都从这一刻（发起切换）算起：从"进入等待"算起会让宽限期实际变短，等于放宽了
    // "早期失败不采信"的范围
    const auto switch_started_at = std::chrono::steady_clock::now();
    esp_err_t err = switch_sta_config(ssid, password);
    if (err != ESP_OK) {
        // 上面已把状态置为 Switching，这里失败也必须回退：状态停住的话配网看门狗
        // 不再按超时判定（它只看 Connecting），STA 模式下就再也不会自动开热点了。
        // NVS 未写、内存未变，STA 配置也保持原值（重连线程继续用旧配置）
        rollback_after_failed_switch(old_ssid, old_password, state_before);
        return err;
    }

    bool rejected = false;
    if (!wait_connection_result(ssid, switch_started_at, rejected)) {
        ESP_LOGW(TAG, "新配置未连上（%s），回滚原配置", rejected ? "AP 拒绝或不在场" : "超时");
        rollback_after_failed_switch(old_ssid, old_password, state_before);
        return rejected ? ESP_FAIL : ESP_ERR_TIMEOUT;
    }

    // 连上才持久化。NVS 写失败只记日志、仍按成功返回：网络已经切过去了，报失败
    // 反而与设备当前状态不符（用户会以为没生效而反复操作）；NVS 写失败只可能是
    // flash 损坏/分区写满这类要重启或维修的情形，调用方没有可执行的补救动作，
    // 日志里已写明"重启后会回退旧配置"
    err = nvs_write_all({{NVS_KEY_SSID, ssid.c_str()}, {NVS_KEY_PASSWORD, password.c_str()}});
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "NVS 写入失败（连接已生效，但重启后会回退旧配置）: %s", esp_err_to_name(err));
    }

    // 内存状态最后更新：等待期间并发读到的仍是旧配置，与"新连接还没成功"的实际
    // 状态一致（成功后两者一起切换）
    bool mode_back_to_sta = false;
    {
        std::lock_guard lock(mutex_);
        ssid_ = ssid;
        password_ = password;
        set_state_locked(WifiState::Connected);
        // 能连上说明用户的期望就是"连 WiFi"：期望模式一并落定，免得在 AP 模式下
        // 配好网、重启后模式仍记着 AP 又不连。这里直接改内存——不能调
        // set_work_mode（它要拿本函数已持有的 apply 锁，同线程会自锁），也不需要
        // 它"切回 Sta 就 reconnect"的动作（此刻已经连着）
        if (work_mode_ == WifiWorkMode::Ap) {
            work_mode_ = WifiWorkMode::Sta;
            mode_back_to_sta = true;
        }
    }
    if (mode_back_to_sta) {
        // 写失败时 persist_work_mode 已记 ERROR，这里补一句用户视角的后果：
        // 重启后会回到"只做热点不连 WiFi"，看起来像配好的网没生效
        if (persist_work_mode(WifiWorkMode::Sta) != ESP_OK) {
            ESP_LOGW(TAG, "重启后仍会是 AP 模式（只做热点），需要再执行一次 wifi_mode sta");
        }
    }
    ESP_LOGI(TAG, "已连上新 AP 并保存配置");
    return ESP_OK;
}

bool WifiConfigManager::wait_connection_result(const std::string &ssid,
                                               std::chrono::steady_clock::time_point switch_started_at,
                                               bool &rejected)
{
    // 连上以"切换后见过一次未关联，之后关联上的又是目标 SSID，且拿到 IP"为准
    // （关联上但 DHCP 失败同样不能用）。两个坑都排在判据里：
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
    rejected = false;
    // 锁只圈住本函数：调用方随后的回滚要重入 switch_sta_config 拿这把锁
    // （非递归锁，同一线程不能重入），NVS 写入等耗时操作也不该压着事件回调
    std::unique_lock wait_lock(wait_mutex_);
    const auto deadline = switch_started_at + std::chrono::seconds(APPLY_TIMEOUT_SECONDS);
    bool saw_disconnected = false;
    // 判据集中成 lambda，循环里查、超时前再查一次（见下）
    const auto connected_to_target = [&]() {
        wifi_ap_record_t ap_info{};
        if (esp_wifi_sta_get_ap_info(&ap_info) != ESP_OK) {
            // 见过一次未关联才算数：旧连接残留期间也报着旧 AP 的信息
            saw_disconnected = true;
            return false;
        }
        // 关联的 SSID 与目标比较：ap_info.ssid 是 33 字节定长数组、末位不一定有
        // 终止符，先按实际长度取出内容再按整串比，不拿定长缓冲去做 strncmp。
        // 找终止符用 std::find 而不是 strnlen：后者是 POSIX 的，IDF 的 <cstring>
        // 不导出 std::strnlen（实测编译不过），std::find 是纯标准且一样简单
        const auto *associated_ptr = reinterpret_cast<const char *>(ap_info.ssid);
        const std::string associated_ssid(
                associated_ptr,
                std::find(associated_ptr, associated_ptr + sizeof(ap_info.ssid), '\0'));
        return saw_disconnected && associated_ssid == ssid && has_ip();
    };
    // 与上面同理抽成 lambda，让超时路径也能复查（见下）。采信条件：确定性失败
    // 要过了宽限期才作数——新配置的失败最早也要几百毫秒（全信道扫描 + 认证）才
    // 可能出现，更早的只能是旧连接迟到的失败事件被处理到了，不能算到新配置头上
    const auto rejected_by_target = [&]() {
        return !connected && saw_disconnected &&
               is_definite_failure(last_disconnect_reason_) &&
               std::chrono::steady_clock::now() - switch_started_at >= kFailureGracePeriod;
    };
    while (true) {
        connected = connected_to_target();
        rejected = rejected_by_target();
        if (connected || rejected) {
            break;
        }
        if (wait_cv_.wait_until(wait_lock, deadline) == std::cv_status::timeout) {
            // 超时前的最后一次确认：结果可能刚好在"解锁与睡眠之间"落定，那次
            // notify 谁也唤不醒——不复查就会把成功误判成超时，"被 AP 拒绝"
            // 也会退化成超时的提示
            connected = connected_to_target();
            rejected = rejected_by_target();
            break;
        }
    }
    return connected;
}

void WifiConfigManager::rollback_after_failed_switch(const std::string &old_ssid,
                                                     const std::string &old_password,
                                                     WifiState state_before)
{
    // 不落 NVS、内存保持旧值、STA 配置也换回旧值，否则重连线程会一直拿刚设进去的
    // （连不上的）新配置重试。
    // 回滚失败（set_config 根本没写进驱动）时只多记一条日志：switch_sta_config
    // 内部已记过驱动错误，这里点明"是回滚这一步失败"，免得排障时只看到一条驱动
    // 错误、推不出"现在新旧配置都没连上"
    const esp_err_t rollback_err = switch_sta_config(old_ssid, old_password);
    if (rollback_err != ESP_OK) {
        ESP_LOGE(TAG, "回滚旧配置也失败（%s）：新旧配置都没连上", esp_err_to_name(rollback_err));
    }
    std::lock_guard lock(mutex_);
    // 回滚后重新计时：旧配置也连不上的话，看门狗到点会把配网热点拉起来。
    // 但切换前热点就开着（用户正在配网页里试新 AP 却失败了）时它并没有被关掉，
    // 得回 ApActive——否则状态说"连接中"，实际热点还在广播
    set_state_locked(state_before == WifiState::ApActive ? WifiState::ApActive
                                                         : WifiState::Connecting);
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
    // 只负责唤醒，不记录状态：是否真的连上了目标 AP 由等待侧按"关联 SSID + 有 IP"
    // 判定（见 apply_config 的等待循环）。
    // 但 notify 仍要拿 wait_mutex_：本事件不改状态、等待侧判断的是网卡/网口的实时
    // 状态，若 notify 与等待侧的"检查 → 进入等待"交错（检查时 IP 还没好、通知恰好
    // 落在解锁与睡眠之间），这次唤醒就丢了，等待侧干等到超时——表现成"明明连上了
    // 却报超时并把连接回滚回旧 AP"。持锁后这个交错不可能发生（与 note_disconnect_reason 同一套）
    std::lock_guard wait_lock(wait_mutex_);
    wait_cv_.notify_all();
}

bool WifiConfigManager::should_start_provisioning_ap()
{
    std::lock_guard lock(mutex_);
    // 期望 AP 模式：热点就是工作方式，还没开就开（不等超时）。STA 侧本来就不连，
    // 状态机停在哪个状态都不该拦住热点
    if (work_mode_ == WifiWorkMode::Ap) {
        return true;
    }
    // Switching 期间不开：正在按新配置切换，结果由 apply_config 收尾
    if (state_ != WifiState::Connecting) {
        return false;
    }
    return std::chrono::steady_clock::now() - state_since_ >= std::chrono::seconds(AP_FALLBACK_SECONDS);
}

WifiConfigManager::WifiWorkMode WifiConfigManager::work_mode()
{
    std::lock_guard lock(mutex_);
    return work_mode_;
}

const char *WifiConfigManager::work_mode_name(WifiWorkMode mode)
{
    return mode == WifiWorkMode::Ap ? NVS_MODE_AP : NVS_MODE_STA;
}

bool WifiConfigManager::parse_work_mode(std::string name, WifiWorkMode &out)
{
    // 大小写不敏感：串口与网页输入 "AP"、"ap" 都认
    for (char &c: name) {
        c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
    }
    if (name == NVS_MODE_STA) {
        out = WifiWorkMode::Sta;
        return true;
    }
    if (name == NVS_MODE_AP) {
        out = WifiWorkMode::Ap;
        return true;
    }
    return false;
}

esp_err_t WifiConfigManager::set_work_mode(WifiWorkMode mode)
{
    // 与 apply_config 串行：配网要等十几秒连接结果，那期间切模式等于插进
    // "切换配置"这个中间态里（结果判定、失败回滚和新模式互相打架），这里排队
    std::lock_guard apply_lock(apply_mutex_);

    {
        std::lock_guard lock(mutex_);
        if (mode == work_mode_) {
            return ESP_OK; // 没变：不写 NVS（有擦写寿命），也不重复做切换动作
        }
        work_mode_ = mode;
    }

    // 与 apply_config 对 NVS 的取舍一致：内存与切换动作都已经生效，写 NVS 失败
    // 只影响"重启后沿用"，返回失败反而与设备当前状态不符（用户以为没切成功、
    // 反复重试；persist_work_mode 内部已记 ERROR 说明重启会回退）
    persist_work_mode(mode);

    if (mode == WifiWorkMode::Ap) {
        // 断开现有连接停止连 WiFi：重连线程下一次 reconnect 会被模式检查挡掉，
        // 配网热点由看门狗随即拉起（Ap 模式下 should_start_provisioning_ap 恒真）。
        // disconnect 特意留在锁外：与 reconnect 的 connect 之间没有有害交错——
        // work_mode_ 的读与写都在 mutex_ 内，只可能有两种顺序：
        // ① reconnect 先拿到锁：它按（当时还是 Sta 的）模式连了一次，随后的
        //    disconnect 把这次多余的连接断掉；
        // ② 本函数先改模式：reconnect 拿锁后看到 Ap 直接返回，不会再有新连接。
        // 两种顺序的终态都是"STA 断开且不再重连"，不必再和 reconnect 互斥
        esp_wifi_disconnect();
    }
    else {
        // 切回连 WiFi：立刻发起连接，不必等下一次断开事件唤醒重连线程
        reconnect();
    }
    ESP_LOGI(TAG, "工作模式已切换为 %s",
             mode == WifiWorkMode::Ap ? "AP（只做配网热点）" : "STA（连接 WiFi）");
    return ESP_OK;
}

esp_err_t WifiConfigManager::persist_work_mode(WifiWorkMode mode)
{
    // 写失败只记日志：内存已按新模式生效，与 apply_config 对 NVS 的取舍一致
    const esp_err_t err = nvs_write_all({{NVS_KEY_MODE, work_mode_name(mode)}});
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "保存工作模式失败（重启后可能仍是旧模式）: %s", esp_err_to_name(err));
    }
    return err;
}

esp_err_t WifiConfigManager::nvs_write_all(
        std::initializer_list<std::pair<const char *, const char *>> items)
{
    nvs_handle_t handle;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &handle);
    if (err != ESP_OK) {
        return err;
    }
    for (const auto &[key, value]: items) {
        err = nvs_set_str(handle, key, value);
        if (err != ESP_OK) {
            break;
        }
    }
    if (err == ESP_OK) {
        err = nvs_commit(handle);
    }
    nvs_close(handle);
    return err;
}

esp_err_t WifiConfigManager::nvs_erase_namespace()
{
    nvs_handle_t handle;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &handle);
    if (err != ESP_OK) {
        return err;
    }
    err = nvs_erase_all(handle);
    if (err == ESP_OK) {
        err = nvs_commit(handle);
    }
    nvs_close(handle);
    return err;
}

void WifiConfigManager::note_sta_connected()
{
    std::lock_guard lock(mutex_);
    set_state_locked(WifiState::Connected);
}

void WifiConfigManager::note_sta_disconnected()
{
    std::lock_guard lock(mutex_);
    // Switching 期间的断开是换配置的过程之一，状态由 apply_config 收尾；
    // 配网模式下 STA 本来就在后台反复重试，每次都把状态拉回 Connecting 会让
    // "热点开着"这件事从状态里消失（看门狗靠 ap_active_ 判断，不会重开，
    // 但状态与事实不符）
    if (state_ == WifiState::Switching || state_ == WifiState::ApActive) {
        return;
    }
    set_state_locked(WifiState::Connecting);
}

void WifiConfigManager::note_ap_started()
{
    std::lock_guard lock(mutex_);
    set_state_locked(WifiState::ApActive);
}

void WifiConfigManager::set_state_locked(WifiState state)
{
    if (state == state_) {
        // 重复进入同一状态不重置计时：重连期间会反复收到断开事件，
        // 每次都重置的话配网热点的超时永远等不到
        return;
    }
    state_ = state;
    state_since_ = std::chrono::steady_clock::now();
    ESP_LOGI(TAG, "WiFi 状态: %s", state_name(state_));
}

esp_err_t WifiConfigManager::reset_config()
{
    // 与 apply_config 串行：配网等结果的十几秒里清配置，会让它的失败回滚拿
    // 一套已经作废的旧值去恢复连接
    std::lock_guard apply_lock(apply_mutex_);

    esp_err_t err = nvs_erase_namespace();
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        // 命名空间不存在 = 本来就没配置过，视为已清空
        err = ESP_OK;
    }
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "清除 NVS 失败: %s", esp_err_to_name(err));
        return err;
    }

    // 回退编译期默认：内存里立即生效，下次开机 load_config 取默认值。
    // 配网热点配置与期望模式也存在同一命名空间里（上面已一并清掉），
    // 内存不回退的话会与 NVS 不一致——热点的名称/密码、工作模式都按清空前的继续用
    WifiWorkMode mode_before = WifiWorkMode::Sta;
    {
        std::lock_guard lock(mutex_);
        mode_before = work_mode_;
        ssid_ = CONFIG_USBIPD_WIFI_SSID;
        password_ = CONFIG_USBIPD_WIFI_PASSWORD;
        ap_ssid_ = CONFIG_USBIPD_AP_SSID;
        ap_password_ = CONFIG_USBIPD_AP_PASSWORD;
        work_mode_ = WifiWorkMode::Sta; // NVS 里没有 mode 记录 = STA（见 load_config）
    }
    // 原来是 AP 模式的话 STA 侧已停摆（reconnect 被模式检查挡掉）、热点却可能
    // 还开着：模式改回 STA 后主动踢一次连接，否则会停在"模式是 STA、STA 不连、
    // 热点一直广播"的僵局，只有重启才恢复。连上后热点自动关（见 GOT_IP 处理）
    if (mode_before == WifiWorkMode::Ap) {
        reconnect();
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

std::string WifiConfigManager::ap_ssid()
{
    std::lock_guard lock(mutex_);
    return ap_ssid_;
}

std::string WifiConfigManager::ap_password()
{
    std::lock_guard lock(mutex_);
    return ap_password_;
}

esp_err_t WifiConfigManager::apply_ap_config(const std::string &ssid, const std::string &password)
{
    // 热点密码要么留空（开放热点），要么满足 WPA2 的 8~63 位；1~7 位 esp_wifi 会拒。
    // 名称与密码都拒绝内嵌 NUL（与 apply_config 同一套规则：串口/脚本直调也要
    // 挡住，内嵌 NUL 会让 NVS 存的串和驱动实际用的截断串不一致）
    const bool ssid_blank = ssid.find_first_not_of(" \t\r\n") == std::string::npos;
    if (ssid.empty() || ssid_blank || ssid.find('\0') != std::string::npos ||
        ssid.size() >= MAX_SSID_LEN || password.find('\0') != std::string::npos ||
        (!password.empty() && (password.size() < 8 || password.size() >= MAX_PASSPHRASE_LEN))) {
        ESP_LOGE(TAG, "非法热点配置: ssid 长度 %zu（1~%u），password 长度 %zu（0 或 8~%u，且不能含空字符）",
                 ssid.size(), static_cast<unsigned>(MAX_SSID_LEN) - 1,
                 password.size(), static_cast<unsigned>(MAX_PASSPHRASE_LEN) - 1);
        return ESP_ERR_INVALID_ARG;
    }

    // 与 apply_config/reset_config 串行：否则会和 reset_config 的"清 NVS + 回退
    // 内存"交错，落成 NVS 与内存各执一词的中间状态
    std::lock_guard apply_lock(apply_mutex_);

    // 先落 NVS 再更新内存（与 apply_config 同样的取舍：持久化失败就当没改过）
    const esp_err_t err = nvs_write_all(
            {{NVS_KEY_AP_SSID, ssid.c_str()}, {NVS_KEY_AP_PASSWORD, password.c_str()}});
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "热点配置写入 NVS 失败: %s", esp_err_to_name(err));
        return err;
    }

    {
        std::lock_guard lock(mutex_);
        ap_ssid_ = ssid;
        ap_password_ = password;
    }
    // 正在运行的热点不重启：改名字/密码会把正连着配网的客户端踢下线，
    // 下次启动热点（下次开机或下次自动进入配网模式）再生效
    ESP_LOGI(TAG, "配网热点配置已保存（下次热点启动时生效）");
    return ESP_OK;
}

void WifiConfigManager::reconnect()
{
    std::lock_guard lock(mutex_);
    // 期望 AP 模式：不主动连 WiFi（配网热点就是当前工作方式），反复重试只会
    // 一直扫描信道、刷日志、耗电
    if (work_mode_ == WifiWorkMode::Ap) {
        return;
    }
    // 与 apply_config 的 set_config/connect 互斥：配置正在更换时在此等锁，等换完
    // 再连（apply_config 自己也会发起连接，这里多连一次是幂等的）
    esp_err_t err = esp_wifi_connect();
    if (err != ESP_OK) {
        ESP_LOGI(TAG, "esp_wifi_connect 返回 %s", esp_err_to_name(err));
    }
}

bool WifiConfigManager::is_connected()
{
    // 关联上了还要有 IP 才算"已连接"：DHCP 没成时两处展示会自相矛盾——
    // /api/status 报 connected=true 却配着 "IP: —"，wifi_show 同理
    wifi_ap_record_t ap_info{};
    return esp_wifi_sta_get_ap_info(&ap_info) == ESP_OK && has_ip();
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
