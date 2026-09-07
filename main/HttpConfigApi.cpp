#include "HttpConfigApi.h"

#include <cctype>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>

#include <esp_log.h>

#include "esp32_handler/Esp32Server.h"
#include "WifiConfigManager.h"
#include "sdkconfig.h"

// 网页以二进制段嵌入固件（main/CMakeLists.txt 的 EMBED_FILES，IDF 标准做法）。
// 符号命名规则见 IDF tools/cmake/scripts/data_file_embed_asm.cmake：
// 文件名的 '.' 转 '_'，起止各一个符号
extern const char index_html_start[] asm("_binary_index_html_start");
extern const char index_html_end[] asm("_binary_index_html_end");

namespace
{

constexpr const char *TAG = "HttpConfigApi";

// POST body 上限：SSID(32)+密码(64) 的 urlencoded 形式绰绰有余，防恶意大包
constexpr std::size_t MAX_FORM_BODY = 512;

// JSON 字符串转义（ssid 可能含引号/反斜杠，直接拼会破坏 JSON）
std::string json_escape(const std::string &in)
{
    std::string out;
    out.reserve(in.size() + 8);
    for (char c: in) {
        if (c == '"' || c == '\\') {
            out.push_back('\\');
        }
        out.push_back(c);
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

// 响应辅助：统一 Content-Type 的 JSON 响应
esp_err_t send_json(httpd_req_t *req, int status, const char *body)
{
    httpd_resp_set_status(req, status == 200 ? "200 OK" : (status == 400 ? "400 Bad Request" : "500 Internal Server Error"));
    httpd_resp_set_type(req, "application/json");
    return httpd_resp_send(req, body, HTTPD_RESP_USE_STRLEN);
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
    const std::string ssid = json_escape(manager.ssid());
    const std::string ip = json_escape(manager.ip_str());

    // 配置口 GPIO 一并下发：配错 WiFi 断网时，页面靠它提示用户接线走串口救急
#if CONFIG_USBIPD_CFG_CONSOLE_ENABLE
    const int console_tx = CONFIG_USBIPD_CFG_UART_TX_GPIO;
    const int console_rx = CONFIG_USBIPD_CFG_UART_RX_GPIO;
#else
    // 配置口编译期未启用：页面据此显示"未启用"而非一串无意义的 GPIO 号
    const int console_tx = -1;
    const int console_rx = -1;
#endif

    char body[320];
    std::snprintf(body, sizeof(body),
                  "{\"connected\":%s,\"ssid\":\"%s\",\"ip\":\"%s\",\"console_tx\":%d,\"console_rx\":%d}",
                  manager.is_connected() ? "true" : "false", ssid.c_str(), ip.c_str(),
                  console_tx, console_rx);
    return send_json(req, 200, body);
}

esp_err_t HttpConfigApi::handle_get_devices(httpd_req_t *req)
{
    // 快照字段由设备自身产生（busid 是端口数字串、vid/pid 是我们格式化的 hex），
    // 不含引号/反斜杠，无 JSON 注入面，直接拼接即可
    std::string body = "{\"devices\":[";
    auto &api = HttpConfigApi::instance();
    if (api.esp32_server_ != nullptr) {
        bool first = true;
        for (const auto &device: api.esp32_server_->list_device_snapshots()) {
            if (!first) {
                body += ',';
            }
            first = false;
            char entry[160];
            std::snprintf(entry, sizeof(entry),
                          "{\"busid\":\"%s\",\"vid\":\"%04x\",\"pid\":\"%04x\",\"in_use\":%s}",
                          device.busid.c_str(), device.vendor_id, device.product_id,
                          device.in_use ? "true" : "false");
            body += entry;
        }
    }
    body += "]}";
    return send_json(req, 200, body.c_str());
}

esp_err_t HttpConfigApi::handle_post_wifi(httpd_req_t *req)
{
    // 只接受小 body：content_len 不可信（可能负/超大），超限直接拒绝
    if (req->content_len <= 0 || static_cast<std::size_t>(req->content_len) > MAX_FORM_BODY) {
        return send_json(req, 400, "{\"error\":\"body too large or empty\"}");
    }

    std::string body(static_cast<std::size_t>(req->content_len), '\0');
    int received = 0;
    while (received < req->content_len) {
        const int n = httpd_req_recv(req, body.data() + received,
                                     static_cast<std::size_t>(req->content_len - received));
        if (n <= 0) {
            // 连接中断等：响应已无法送达，直接返回即可
            return ESP_FAIL;
        }
        received += n;
    }

    std::string ssid;
    std::string password;
    if (!form_value(body, "ssid", ssid) || ssid.empty()) {
        return send_json(req, 400, "{\"error\":\"ssid required\"}");
    }
    form_value(body, "password", password); // 缺失/空 = 开放 AP

    auto &manager = WifiConfigManager::instance();
    esp_err_t err = manager.apply_config(ssid, password);
    if (err != ESP_OK) {
        char error[96];
        std::snprintf(error, sizeof(error), "{\"error\":\"%s\"}", esp_err_to_name(err));
        return send_json(req, 400, error);
    }

    // 应用后当前连接会被断开重连，返回成功提示即可
    return send_json(req, 200, "{\"result\":\"ok, reconnecting\"}");
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

    for (const httpd_uri_t *uri: {&index_uri, &status_uri, &devices_uri, &wifi_uri}) {
        err = httpd_register_uri_handler(server_, uri);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "注册 %s 失败: %s", uri->uri, esp_err_to_name(err));
            return err;
        }
    }

    ESP_LOGI(TAG, "HTTP 配置服务已启动: 端口 %d", config.server_port);
    return ESP_OK;
}
