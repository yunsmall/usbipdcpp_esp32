#include "HttpConfigApi.h"

#include <cctype>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>

#include <esp_log.h>
#include <esp_wifi.h>

#include "esp32_handler/Esp32Server.h"
#include "WifiConfigManager.h"
#include "WifiConnection.h"
#include "sdkconfig.h"

// 网页以二进制段嵌入固件（main/CMakeLists.txt 的 EMBED_FILES，IDF 标准做法）。
// 符号命名规则见 IDF tools/cmake/scripts/data_file_embed_asm.cmake：先取
// **文件名**（get_filename_component(... NAME)，目录部分在这一步就丢掉了），
// 再做 C 标识符转换——web/index.html 的符号是 index_html，不带 "web" 这一截
extern const char index_html_start[] asm("_binary_index_html_start");
extern const char index_html_end[] asm("_binary_index_html_end");

namespace
{

constexpr const char *TAG = "HttpConfigApi";

// POST body 上限：SSID(32)+密码(64) 的 urlencoded 形式绰绰有余，防恶意大包
constexpr std::size_t MAX_FORM_BODY = 512;

// JSON 字符串转义：ssid 可能含引号/反斜杠/控制字符，直接拼会破坏 JSON。
// 控制字符必须转成 \uXXXX——JSON 字符串里不允许出现裸控制字符，浏览器
// JSON.parse 遇到会直接抛错，整个 /api/status 就都读不出来了
std::string json_escape(const std::string &in)
{
    std::string out;
    // 每个控制字符会膨胀成 \uXXXX（6 字节），按最坏情况预留，省掉中途几次扩容
    out.reserve(in.size() * 6);
    for (char c: in) {
        const auto uc = static_cast<unsigned char>(c);
        if (c == '"' || c == '\\') {
            out.push_back('\\');
            out.push_back(c);
        }
        else if (uc < 0x20) {
            char esc[8];
            std::snprintf(esc, sizeof(esc), "\\u%04x", uc);
            out += esc;
        }
        else {
            out.push_back(c);
        }
    }
    return out;
}

// 百分号解码一段（form-urlencoded 的 + 表示空格）。长度不足时静默丢弃超长部分
void percent_decode_into(const std::string &in, std::string &out)
{
    for (std::size_t i = 0; i < in.size(); i++) {
        if (in[i] == '+') {
            out.push_back(' ');
        }
        else if (in[i] == '%' && i + 2 < in.size() &&
                 std::isxdigit(static_cast<unsigned char>(in[i + 1])) &&
                 std::isxdigit(static_cast<unsigned char>(in[i + 2]))) {
            out.push_back(static_cast<char>(std::strtoul(in.substr(i + 1, 2).c_str(), nullptr, 16)));
            i += 2;
        }
        else {
            out.push_back(in[i]);
        }
    }
}

// 从 form-urlencoded body 取 key 对应的值（body 形如 "ssid=..&password=.."）
bool form_value(const std::string &body, const char *key, std::string &out)
{
    const std::string prefix = std::string(key) + "=";
    std::size_t pos = 0;
    while (pos <= body.size()) {
        const std::size_t amp = body.find('&', pos);
        const std::string segment = body.substr(pos, amp == std::string::npos ? std::string::npos : amp - pos);
        if (segment.rfind(prefix, 0) == 0) {
            const std::string encoded = segment.substr(prefix.size());
            percent_decode_into(encoded, out);
            return true;
        }
        if (amp == std::string::npos) {
            break;
        }
        pos = amp + 1;
    }
    return false;
}

// 响应辅助：统一 Content-Type 的 JSON 响应。状态行用静态字符串表——httpd_resp_set_status
// 只保存指针不复制（httpd_txrx.c 里就是 ra->status = status），传栈上缓冲或临时
// string 会留下悬垂指针；用表还顺带把 404/405 之类的码映射对，不会静默变 500
esp_err_t send_json(httpd_req_t *req, int status, const char *body)
{
    const char *status_line = "500 Internal Server Error";
    switch (status) {
        case 200:
            status_line = "200 OK";
            break;
        case 400:
            status_line = "400 Bad Request";
            break;
        case 404:
            status_line = "404 Not Found";
            break;
        case 405:
            status_line = "405 Method Not Allowed";
            break;
        default:
            // 状态行必须是静态字符串（httpd_resp_set_status 只存指针不复制，见上），
            // 没映射的码只能落到 500 行：说一声，免得将来加新码时静默用错状态行
            ESP_LOGW(TAG, "send_json 未映射的状态码 %d，按 500 返回", status);
            break;
    }
    httpd_resp_set_status(req, status_line);
    httpd_resp_set_type(req, "application/json");
    return httpd_resp_send(req, body, HTTPD_RESP_USE_STRLEN);
}

// 读取请求 body（form-urlencoded）：ESP_OK=读好了；ESP_ERR_INVALID_SIZE=体积
// 非法（400 已回复）；ESP_FAIL=连接中断（响应已无法送达）——后两者调用方直接
// 返回对应错误码即可
esp_err_t read_form_body(httpd_req_t *req, std::string &body)
{
    // 只接受非空且不大的 body：content_len 为 0 或超出上限都直接拒绝（写成 <= 0
    // 是防御写法，该字段现为 size_t，与 == 0 等价）
    if (req->content_len <= 0 || static_cast<std::size_t>(req->content_len) > MAX_FORM_BODY) {
        send_json(req, 400, "{\"error\":\"body too large or empty\"}");
        return ESP_ERR_INVALID_SIZE;
    }

    // 定长读取：content_len 之后就是要收的全部内容
    body.assign(static_cast<std::size_t>(req->content_len), '\0');
    int received = 0;
    while (received < req->content_len) {
        const int n = httpd_req_recv(req, body.data() + received,
                                     static_cast<std::size_t>(req->content_len - received));
        if (n <= 0) {
            return ESP_FAIL;
        }
        received += n;
    }
    return ESP_OK;
}

} // anonymous namespace

HttpConfigApi &HttpConfigApi::instance()
{
    static HttpConfigApi api;
    return api;
}

void HttpConfigApi::set_server(usbipdcpp::Esp32Server *server)
{
    esp32_server_ = server;
}

esp_err_t HttpConfigApi::handle_get_index(httpd_req_t *req)
{
    // 网页本体在 main/web/index.html（独立文件便于维护与预览），编译期嵌入固件
    httpd_resp_set_type(req, "text/html; charset=utf-8");
    return httpd_resp_send(req, index_html_start, index_html_end - index_html_start);
}

esp_err_t HttpConfigApi::handle_get_status(httpd_req_t *req)
{
    auto &manager = WifiConfigManager::instance();

    // 配置口 GPIO 一并下发：配错 WiFi 断网时，页面靠它提示用户接线走串口救急
#if CONFIG_USBIPD_CFG_CONSOLE_ENABLE
    const int console_tx = CONFIG_USBIPD_CFG_UART_TX_GPIO;
    const int console_rx = CONFIG_USBIPD_CFG_UART_RX_GPIO;
#else
    // 配置口编译期未启用：页面据此显示"未启用"而非一串无意义的 GPIO 号
    const int console_tx = -1;
    const int console_rx = -1;
#endif

    // ap_active/ap_ssid：页面在配网模式下（连的就是设备热点）要显示热点信息。
    // ap_auth 只报"有没有密码"不下发密码本身——页面据此提示"改名称时密码框
    // 留空会把热点改成开放的"，避免在页面上泄露已有的密码
    // 留 768 而不是紧凑值：两个 SSID 都可能含控制字符，json_escape 按 \uXXXX
    // 转义时单字节膨胀到 6 倍（最坏 31*6*2 + 固定部分 ≈ 530 字节）
    // 字符串字段一律在拼接处就地转义（改成预先转义过的变量极易被再转一次，
    // 显示出来就多一层反斜杠）：每个值只过一次 json_escape，一次都不能多
    char body[768];
    const int body_len = std::snprintf(body, sizeof(body),
                  "{\"connected\":%s,\"ssid\":\"%s\",\"ip\":\"%s\",\"console_tx\":%d,\"console_rx\":%d,"
                  "\"ap_active\":%s,\"ap_ssid\":\"%s\",\"ap_auth\":%s,\"work_mode\":\"%s\"}",
                  manager.is_connected() ? "true" : "false",
                  json_escape(manager.ssid()).c_str(), json_escape(manager.ip_str()).c_str(),
                  console_tx, console_rx,
                  WifiConnection::instance().is_ap_active() ? "true" : "false",
                  json_escape(manager.ap_ssid()).c_str(),
                  manager.ap_password().empty() ? "false" : "true",
                  WifiConfigManager::work_mode_name(manager.work_mode()));
    if (body_len < 0 || static_cast<std::size_t>(body_len) >= sizeof(body)) {
        // 以后加字段或 SSID 全是待转义字符时可能顶到上限：宁可报错，也别把截断
        // 的非法 JSON 发给页面（那会让整页状态读不出来，比 500 更难排查）
        ESP_LOGE(TAG, "status JSON 超出缓冲: %d 字节", body_len);
        return send_json(req, 500, "{\"error\":\"status too large\"}");
    }
    return send_json(req, 200, body);
}

esp_err_t HttpConfigApi::handle_get_devices(httpd_req_t *req)
{
    // vid/pid 是我们自己格式化的 hex、in_use 是字面量 true/false，没有注入面；
    // busid 当前是设备侧生成的端口串（"1-1" 这类），仍走一次转义：万一它将来
    // 带上引号/反斜杠，拼出来的就不是合法 JSON，整个设备列表都会解析失败
    std::string body = "{\"devices\":[";
    // 一次 load 出指针再用：httpd 是多任务，判断与遍历之间不该重复解引用
    auto *server = instance().esp32_server_.load();
    if (server != nullptr) {
        // 取一次快照：预分配与遍历都用它（range-for 里直接调也只求值一次，这里
        // 显式取出来是为了按设备数预留缓冲，省掉拼串时的反复扩容）
        const auto snapshots = server->list_device_snapshots();
        body.reserve(64 + 128 * snapshots.size());
        bool first = true;
        for (const auto &device: snapshots) {
            // 先格式化再决定分隔符：条目过长要整条跳过，不能留下一个光秃秃的逗号
            const std::string busid = json_escape(device.busid);
            char entry[256];
            const int n = std::snprintf(entry, sizeof(entry),
                                        "{\"busid\":\"%s\",\"vid\":\"%04x\",\"pid\":\"%04x\",\"in_use\":%s}",
                                        busid.c_str(), static_cast<unsigned>(device.vendor_id),
                                        static_cast<unsigned>(device.product_id),
                                        device.in_use ? "true" : "false");
            if (n < 0 || static_cast<std::size_t>(n) >= sizeof(entry)) {
                // 截断的话页面上显示的是错的 busid，拿它 attach 必然失败，
                // 不如整条不报（busid 由设备侧生成，正常是 "1-1" 这种短串）
                ESP_LOGW(TAG, "设备条目过长被截断，跳过: busid=%s", device.busid.c_str());
                continue;
            }
            if (!first) {
                body += ',';
            }
            first = false;
            body += entry;
        }
    }
    body += "]}";
    return send_json(req, 200, body.c_str());
}

esp_err_t HttpConfigApi::handle_post_wifi(httpd_req_t *req)
{
    std::string body;
    const esp_err_t read_err = read_form_body(req, body);
    if (read_err != ESP_OK) {
        // 体积非法时 400 已经回复过了：这里对 httpd 报"处理完毕"，返回非 ESP_OK
        // 会被当成 handler 失败、再去关连接/记错误；连接真断了才原样返回 ESP_FAIL
        return read_err == ESP_ERR_INVALID_SIZE ? ESP_OK : read_err;
    }

    std::string ssid;
    std::string password;
    if (!form_value(body, "ssid", ssid) || ssid.empty()) {
        return send_json(req, 400, "{\"error\":\"ssid required\"}");
    }
    // 全空白等同未填、内嵌 NUL 则是"存下来的串和实际用的不是一个"：apply_config
    // 也会拒，但那会走到 200 + "配置未能应用"，脚本直调看不出是入参问题
    if (ssid.find_first_not_of(" \t\r\n") == std::string::npos ||
        ssid.find('\0') != std::string::npos) {
        return send_json(req, 400, "{\"error\":\"ssid required\"}");
    }
    form_value(body, "password", password); // 缺失/空 = 开放 AP

    // 内嵌 NUL 先查（只可能来自手工构造的 %00）：它比长度问题更根本——含 NUL 的
    // 密码无论多长都不合法，先报"含空字符"比先报"长度不对"更贴近真正的原因。
    // 放过去的话 NVS 与驱动按 C 字符串截断，存的和连的不是同一串，还得回头查
    if (password.find('\0') != std::string::npos) {
        return send_json(req, 400, "{\"error\":\"密码不能包含空字符\"}");
    }
    // 参数预检（与 WifiConfigManager::apply_config 相同的长度规则）：长度是请求
    // 本身的错，返回 400 比让 apply_config 报"配置未能应用"更清楚
    if (ssid.size() >= MAX_SSID_LEN || password.size() >= MAX_PASSPHRASE_LEN) {
        return send_json(req, 400, "{\"error\":\"ssid/password too long\"}");
    }
    // 非空密码要够 WPA2 的 8 位（空 = 开放网络）：不够的交给驱动只会在连接阶段
    // 失败，报出来的错会含糊成"密码错误或找不到 AP"
    if (!password.empty() && password.size() < 8) {
        return send_json(req, 400, "{\"error\":\"密码至少 8 位（开放网络请留空）\"}");
    }

    auto &manager = WifiConfigManager::instance();
    // 同步等连接结果（最长 APPLY_TIMEOUT_SECONDS 秒）：连上才算成功、才保存进
    // NVS，页面据此显示"连接中"并展示结果。换 AP 后本机 IP 变化，这条响应可能
    // 送不到旧连接（页面端有兜底提示）；只要响应送达，结果一定是准的
    esp_err_t err = manager.apply_config(ssid, password);
    if (err == ESP_OK) {
        // 响应另起名字：上面的 body 是请求体，同名会遮蔽（读起来容易看错）
        char resp[96];
        std::snprintf(resp, sizeof(resp), "{\"ok\":true,\"ip\":\"%s\"}",
                      json_escape(manager.ip_str()).c_str());
        return send_json(req, 200, resp);
    }
    if (err == ESP_ERR_TIMEOUT) {
        char resp[128];
        std::snprintf(resp, sizeof(resp), "{\"ok\":false,\"error\":\"连接超时（%d 秒内未连上）\"}",
                      WifiConfigManager::APPLY_TIMEOUT_SECONDS);
        return send_json(req, 200, resp);
    }
    if (err == ESP_FAIL) {
        return send_json(req, 200, "{\"ok\":false,\"error\":\"密码错误或找不到该 AP\"}");
    }
    ESP_LOGE(TAG, "应用 WiFi 配置失败: %s", esp_err_to_name(err));
    return send_json(req, 200, "{\"ok\":false,\"error\":\"配置未能应用\"}");
}

esp_err_t HttpConfigApi::handle_post_ap(httpd_req_t *req)
{
    std::string body;
    const esp_err_t read_err = read_form_body(req, body);
    if (read_err != ESP_OK) {
        // 体积非法时 400 已经回复过了：这里对 httpd 报"处理完毕"，返回非 ESP_OK
        // 会被当成 handler 失败、再去关连接/记错误；连接真断了才原样返回 ESP_FAIL
        return read_err == ESP_ERR_INVALID_SIZE ? ESP_OK : read_err;
    }

    std::string ssid;
    std::string password;
    if (!form_value(body, "ssid", ssid) || ssid.empty()) {
        return send_json(req, 400, "{\"error\":\"ssid required\"}");
    }
    // 同 handle_post_wifi：全空白与内嵌 NUL 都按入参错误提前挡掉
    if (ssid.find_first_not_of(" \t\r\n") == std::string::npos ||
        ssid.find('\0') != std::string::npos) {
        return send_json(req, 400, "{\"error\":\"ssid required\"}");
    }
    form_value(body, "password", password); // 缺失/空 = 开放热点
    if (password.find('\0') != std::string::npos) {
        return send_json(req, 400, "{\"error\":\"密码不能包含空字符\"}");
    }

    // 与 handle_post_wifi 对称的预检：同样的规则在入口先报 400，别让正常输错
    // 走到 apply_ap_config 里去记一条 ERROR 日志
    if (ssid.size() >= MAX_SSID_LEN ||
        (!password.empty() && (password.size() < 8 || password.size() >= MAX_PASSPHRASE_LEN))) {
        return send_json(req, 400, "{\"error\":\"热点名称 1~31 字符；密码留空（开放）或 8~63 位\"}");
    }

    // 名称/密码只存配置、不动正在运行的热点：此刻连着热点的就是正在配网的人，
    // 把他踢下线没有意义；新名称/密码下次热点启动时生效（见 apply_ap_config）
    const esp_err_t err = WifiConfigManager::instance().apply_ap_config(ssid, password);
    if (err == ESP_ERR_INVALID_ARG) {
        return send_json(req, 400, "{\"error\":\"热点名称 1~31 字符；密码留空（开放）或 8~63 位\"}");
    }
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "保存配网热点配置失败: %s", esp_err_to_name(err));
        return send_json(req, 200, "{\"ok\":false,\"error\":\"保存失败\"}");
    }
    return send_json(req, 200, "{\"ok\":true}");
}

esp_err_t HttpConfigApi::handle_post_mode(httpd_req_t *req)
{
    std::string body;
    const esp_err_t read_err = read_form_body(req, body);
    if (read_err != ESP_OK) {
        // 体积非法时 400 已经回复过了：这里对 httpd 报"处理完毕"，返回非 ESP_OK
        // 会被当成 handler 失败、再去关连接/记错误；连接真断了才原样返回 ESP_FAIL
        return read_err == ESP_ERR_INVALID_SIZE ? ESP_OK : read_err;
    }

    std::string mode;
    WifiConfigManager::WifiWorkMode work_mode;
    if (!form_value(body, "mode", mode) || !WifiConfigManager::parse_work_mode(mode, work_mode)) {
        return send_json(req, 400, "{\"error\":\"mode must be sta or ap\"}");
    }
    const esp_err_t err = WifiConfigManager::instance().set_work_mode(work_mode);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "设置工作模式失败: %s", esp_err_to_name(err));
        return send_json(req, 200, "{\"ok\":false,\"error\":\"模式保存失败\"}");
    }
    return send_json(req, 200, "{\"ok\":true}");
}

esp_err_t HttpConfigApi::init()
{
#if !CONFIG_USBIPD_HTTPD_ENABLE
    return ESP_OK;
#endif

    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.server_port = CONFIG_USBIPD_HTTPD_PORT;
    // 页面轮询最多同时两条 fetch，用不到默认 7 个并发连接。lwip 的 fd 与
    // 活动 TCP 名额有限（usbip 会话与 asio 中断管道也在抢），收紧 httpd 占用
    config.max_open_sockets = 4;
    // handler 里会调 esp_wifi 系列 API（栈消耗高于纯 IO handler），加大栈
    config.stack_size = 8192;
    // 长连接/慢客户端占满 socket 时按 LRU 踢掉空闲连接，保证新配置请求能进来
    config.lru_purge_enable = true;

    esp_err_t err = httpd_start(&server_, &config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "httpd 启动失败（端口 %d）: %s", config.server_port, esp_err_to_name(err));
        return err;
    }

    const httpd_uri_t index_uri = {
            .uri = "/",
            .method = HTTP_GET,
            .handler = &HttpConfigApi::handle_get_index,
            .user_ctx = nullptr,
    };
    const httpd_uri_t status_uri = {
            .uri = "/api/status",
            .method = HTTP_GET,
            .handler = &HttpConfigApi::handle_get_status,
            .user_ctx = nullptr,
    };
    const httpd_uri_t devices_uri = {
            .uri = "/api/devices",
            .method = HTTP_GET,
            .handler = &HttpConfigApi::handle_get_devices,
            .user_ctx = nullptr,
    };
    const httpd_uri_t wifi_uri = {
            .uri = "/api/wifi",
            .method = HTTP_POST,
            .handler = &HttpConfigApi::handle_post_wifi,
            .user_ctx = nullptr,
    };
    const httpd_uri_t ap_uri = {
            .uri = "/api/ap",
            .method = HTTP_POST,
            .handler = &HttpConfigApi::handle_post_ap,
            .user_ctx = nullptr,
    };
    const httpd_uri_t mode_uri = {
            .uri = "/api/mode",
            .method = HTTP_POST,
            .handler = &HttpConfigApi::handle_post_mode,
            .user_ctx = nullptr,
    };

    for (const httpd_uri_t *uri: {&index_uri, &status_uri, &devices_uri, &wifi_uri, &ap_uri, &mode_uri}) {
        err = httpd_register_uri_handler(server_, uri);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "注册 %s 失败: %s", uri->uri, esp_err_to_name(err));
            // 起了一半的服务比没起更麻烦：路由残缺、socket 白占，整个停掉
            httpd_stop(server_);
            server_ = nullptr;
            return err;
        }
    }

    ESP_LOGI(TAG, "HTTP 配置服务已启动: 端口 %d", config.server_port);
    return ESP_OK;
}
