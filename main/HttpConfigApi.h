#pragma once

#include <esp_err.h>
#include <esp_http_server.h>

namespace usbipdcpp
{
    class Esp32Server;
}

/**
 * @brief WiFi 配置与设备状态的 HTTP REST 接口（esp_http_server）
 *
 * 有网时通过浏览器/脚本改 WiFi 配置。绑定 0.0.0.0 启动一次即可：WiFi 断开/重连
 * 不影响监听 socket，lwip 会自动把新 IP 路由到同一 socket（无需按 IP 事件重启）。
 *
 * 路由：
 *   GET  /            网页（main/web/index.html，EMBED_FILES 嵌入，见 CMakeLists）
 *   GET  /api/status  {"connected":..,"ssid":"..","stored":..,"ip":".."}
 *   GET  /api/devices {"devices":[{"busid":"1-1","vid":"046d","pid":"c077","in_use":..},..]}
 *                     设备列表与占用状态（由 esp32_usbipdcpp.cpp 挂接的 Esp32Server 提供）
 *   POST /api/wifi    form-urlencoded: ssid=..&password=..（空 password=开放 AP），
 *                     成功后应用新配置，当前连接会被断开重连
 */
class HttpConfigApi
{
public:
    static HttpConfigApi &instance();

    HttpConfigApi(const HttpConfigApi &) = delete;
    HttpConfigApi &operator=(const HttpConfigApi &) = delete;

    /**
     * @brief 启动 HTTP 服务并注册路由。应在 WiFi 初始化完成后调用；
     *        失败（端口被占等）只记日志返回错误码，不影响主流程
     */
    esp_err_t init();

    /**
     * @brief 挂接 usbip 服务器实例供 /api/devices 查询设备状态。
     *        由启动流程在 server.start 成功后调用；实例生命周期需覆盖本服务
     *        （thread_main 栈上对象，进程存活期有效）
     */
    void set_server(usbipdcpp::Esp32Server *server);

private:
    HttpConfigApi() = default;

    static esp_err_t handle_get_index(httpd_req_t *req);
    static esp_err_t handle_get_status(httpd_req_t *req);
    static esp_err_t handle_get_devices(httpd_req_t *req);
    static esp_err_t handle_post_wifi(httpd_req_t *req);

    httpd_handle_t server_ = nullptr;

    // 挂接的 usbip 服务器（nullptr = 尚未挂接，/api/devices 返回空列表）
    usbipdcpp::Esp32Server *esp32_server_ = nullptr;
};
