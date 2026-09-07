#include "ConfigConsole.h"

#include <atomic>
#include <cstdio>
#include <cstring>
#include <string>

#include <argtable3/argtable3.h>
#include <driver/uart.h>
#include <esp_console.h>
#include <esp_heap_caps.h>
#include <esp_log.h>
#include <esp_system.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <spdlog/spdlog.h>
#include <spdlog/sinks/base_sink.h>
#include <spdlog/pattern_formatter.h>

#include "esp32_handler/Esp32Server.h"
#include "WifiConfigManager.h"
#include "sdkconfig.h"

namespace
{

constexpr const char *TAG = "ConfigConsole";

// 日志镜像总开关：esp_log hook 与 spdlog sink 都检查它，logs 命令翻转
std::atomic<bool> s_log_mirror_enabled{false};

// 行尾规范化后写配置口（esp_log hook 路径专用；spdlog 路径见 init 里镜像 sink
// 的 formatter eol 说明，不经过这里）。esp_log 的格式串行尾是裸 \n（无 \r），
// 真实 UART 终端（CH340 + putty/minicom 一类不做 LF 回车转换）会呈阶梯状，
// 孤立 \n 前需补 \r。
// 实现刻意不引第二缓冲：大栈数组会把 nio 等小栈线程压爆（日志链路跑在调用
// 线程栈上，实测 512B 数组 + 4KB 栈溢出）；堆分配则怕长期运行碎片化。
// 无孤立 \n 时整行一次写，有则拆小段直写，补 \r 处只多一次两字节写
void uart_write_normalized(uart_port_t port, const char *data, std::size_t len)
{
    std::size_t seg_start = 0;
    for (std::size_t i = 0; i < len; i++) {
        if (data[i] == '\n' && (i == 0 || data[i - 1] != '\r')) {
            if (i > seg_start) {
                uart_write_bytes(port, data + seg_start, i - seg_start);
            }
            static constexpr char crlf[] = "\r\n";
            uart_write_bytes(port, crlf, sizeof(crlf) - 1);
            seg_start = i + 1;
        }
    }
    if (seg_start < len) {
        uart_write_bytes(port, data + seg_start, len - seg_start);
    }
}

/**
 * @brief 写配置口 UART 的 spdlog sink
 *
 * 只挂在 default logger 的 sinks 上一次（init 阶段，见 ConfigConsole::init 注释），
 * 运行中只翻转 s_log_mirror_enabled，不做任何增删——sinks() 是裸引用 vector，
 * 运行时增删与并发打日志是 data race（spdlog logger 无锁遍历）
 */
class UartMirrorSink final : public spdlog::sinks::base_sink<std::mutex>
{
public:
    explicit UartMirrorSink(uart_port_t port) : port_(port) {}

protected:
    void sink_it_(const spdlog::details::log_msg &msg) override
    {
        if (!s_log_mirror_enabled.load(std::memory_order_relaxed)) {
            return;
        }
        spdlog::memory_buf_t formatted;
        // 行尾已是 \r\n（init 里给本 sink 配了 eol="\r\n" 的 formatter），
        // 输出无需二次规范化，整行一次写出
        formatter_->format(msg, formatted);
        // REPL 启动后 driver 必已安装（esp_console_new_repl_uart 内部安装）；
        // 失败静默丢弃，不影响主日志通道
        uart_write_bytes(port_, formatted.data(), formatted.size());
    }

    void flush_() override {}

private:
    uart_port_t port_;
};

// 配置口 UART 号（由 ConfigConsole::init 从 Kconfig 填入，供 hook/sink 使用）
uart_port_t s_config_uart_port = static_cast<uart_port_t>(CONFIG_USBIPD_CFG_UART_NUM);

} // anonymous namespace

// 与头文件 static 成员定义一一对应
vprintf_like_t ConfigConsole::s_orig_log_vprintf = nullptr;

ConfigConsole &ConfigConsole::instance()
{
    static ConfigConsole console;
    return console;
}

int ConfigConsole::mirror_log_vprintf(const char *fmt, va_list args)
{
    // UART0 输出保持原样：调 esp_log_set_vprintf 返回的原实现
    int ret = 0;
    if (s_orig_log_vprintf != nullptr) {
        va_list args_uart0;
        va_copy(args_uart0, args);
        ret = s_orig_log_vprintf(fmt, args_uart0);
        va_end(args_uart0);
    }

    if (s_log_mirror_enabled.load(std::memory_order_relaxed)) {
        va_list args_mirror;
        va_copy(args_mirror, args);
        char buf[256];
        const int len = std::vsnprintf(buf, sizeof(buf), fmt, args_mirror);
        va_end(args_mirror);
        if (len > 0) {
            // 镜像口写满整行（一行日志不会超过 255 字节，截断只影响单行显示）
            const std::size_t to_write = static_cast<std::size_t>(len) < sizeof(buf)
                                                 ? static_cast<std::size_t>(len)
                                                 : sizeof(buf) - 1;
            uart_write_normalized(s_config_uart_port, buf, to_write);
        }
    }
    return ret;
}

void ConfigConsole::set_log_mirror(bool enabled)
{
    s_log_mirror_enabled.store(enabled, std::memory_order_relaxed);
}

void ConfigConsole::set_server(usbipdcpp::Esp32Server *server)
{
    esp32_server_ = server;
}

usbipdcpp::Esp32Server *ConfigConsole::server() const
{
    return esp32_server_;
}

// ========== REPL 命令（argtable3 声明参数，esp_console 惯例见 IDF cmd_nvs 示例） ==========
namespace
{

// wifi_set：两个位置参数，password 可选（缺省 = 开放 AP）
struct WifiSetArgs
{
    struct arg_str *ssid;
    struct arg_str *password;
    struct arg_end *end;
};

WifiSetArgs s_wifi_set_args = {};

// wifi_show / wifi_reset / logs / about / devices / mem：无参数命令也带 argtable
// （只含 arg_end），解析器会拦截多余参数并打印 usage
struct WifiShowArgs
{
    struct arg_end *end;
};

WifiShowArgs s_wifi_show_args = {};
WifiShowArgs s_wifi_reset_args = {};
WifiShowArgs s_logs_args = {};
WifiShowArgs s_about_args = {};
WifiShowArgs s_devices_args = {};
WifiShowArgs s_mem_args = {};

int cmd_wifi_set(int argc, char **argv)
{
    const int nerrors = arg_parse(argc, argv, reinterpret_cast<void **>(&s_wifi_set_args));
    if (nerrors != 0) {
        arg_print_errors(stderr, s_wifi_set_args.end, argv[0]);
        return 1;
    }
    // 密码含空格时用双引号包裹或反斜杠转义输入（esp_console 分词器支持，
    // 与 bash 规则一致）
    const char *password = s_wifi_set_args.password->count > 0 ? s_wifi_set_args.password->sval[0] : "";
    esp_err_t err = WifiConfigManager::instance().apply_config(s_wifi_set_args.ssid->sval[0], password);
    if (err != ESP_OK) {
        printf("设置失败: %s\n", esp_err_to_name(err));
        return 1;
    }
    printf("已设置并触发重连，重启后仍生效\n");
    return 0;
}

int cmd_wifi_show(int argc, char **argv)
{
    const int nerrors = arg_parse(argc, argv, reinterpret_cast<void **>(&s_wifi_show_args));
    if (nerrors != 0) {
        arg_print_errors(stderr, s_wifi_show_args.end, argv[0]);
        return 1;
    }
    auto &manager = WifiConfigManager::instance();
    const std::string ssid = manager.ssid();
    const std::string ip = manager.ip_str();
    printf("当前配置 SSID: %s\n", ssid.c_str());
    printf("密码: %s\n", manager.password().empty() ? "<空（开放网络）>" : "********");
    printf("连接状态: %s\n", manager.is_connected() ? "已连接" : "未连接");
    printf("IP: %s\n", ip.empty() ? "<无>" : ip.c_str());
    return 0;
}

int cmd_wifi_reset(int argc, char **argv)
{
    const int nerrors = arg_parse(argc, argv, reinterpret_cast<void **>(&s_wifi_reset_args));
    if (nerrors != 0) {
        arg_print_errors(stderr, s_wifi_reset_args.end, argv[0]);
        return 1;
    }
    esp_err_t err = WifiConfigManager::instance().reset_config();
    if (err != ESP_OK) {
        printf("重置失败: %s\n", esp_err_to_name(err));
        return 1;
    }
    printf("已清除 NVS 记录，当前内存配置回退编译期默认（重启后生效），当前连接不变\n");
    return 0;
}

int cmd_logs(int argc, char **argv)
{
    const int nerrors = arg_parse(argc, argv, reinterpret_cast<void **>(&s_logs_args));
    if (nerrors != 0) {
        arg_print_errors(stderr, s_logs_args.end, argv[0]);
        return 1;
    }
    // REPL 命令执行期间不处理输入（阻塞在 esp_console_run），ctrl-C 只留在 RX 缓冲，
    // 因此本命令自行轮询配置口 RX 收 0x03 作为退出信号
    uart_flush_input(s_config_uart_port);
    ConfigConsole::set_log_mirror(true);
    printf("日志镜像已开启（UART0 日志同时显示在此口），按 Ctrl-C 停止\n");

    uint8_t byte = 0;
    while (true) {
        // 200ms 轮询间隔保证停止指令的及时性，也避免空转占 CPU
        const int n = uart_read_bytes(s_config_uart_port, &byte, 1, pdMS_TO_TICKS(200));
        if (n > 0 && byte == 0x03) {
            break;
        }
    }

    ConfigConsole::set_log_mirror(false);
    // 清掉镜像期间用户可能误输入的残余字节，避免下一条命令被吞首字符
    uart_flush_input(s_config_uart_port);
    printf("\n日志镜像已停止\n");
    return 0;
}

// 固件简介：配置口多为首次上手/忘了用途的场景，help 只列命令名不够直观
int cmd_about(int argc, char **argv)
{
    const int nerrors = arg_parse(argc, argv, reinterpret_cast<void **>(&s_about_args));
    if (nerrors != 0) {
        arg_print_errors(stderr, s_about_args.end, argv[0]);
        return 1;
    }
    printf(
            "usbipdcpp_esp32：ESP32 上的 USB/IP 服务器固件。\n"
            "把插在 ESP32 USB Host 口上的 USB 设备（键鼠/存储/串口等）经 WiFi 网络\n"
            "共享出去，电脑端用 USB/IP 客户端（Windows: usbip-win2；Linux: usbip）\n"
            "attach 到本机后即可像本地设备一样使用，服务器监听 TCP 3240。\n"
            "管理接口：\n"
            "  help            查看全部命令\n"
            "  wifi_set/show/reset  配网（存 NVS，断电不丢）\n"
            "  logs            实时查看主串口日志，Ctrl-C 退出\n"
            "联网后也可用浏览器打开 http://<本机IP>/ 页面配网\n");
    return 0;
}

// 列出已接入设备：与网页设备面板同源（Esp32Server::list_device_snapshots），
// 拔插 USB 设备或远端 attach/断开后内容即时变化
int cmd_devices(int argc, char **argv)
{
    const int nerrors = arg_parse(argc, argv, reinterpret_cast<void **>(&s_devices_args));
    if (nerrors != 0) {
        arg_print_errors(stderr, s_devices_args.end, argv[0]);
        return 1;
    }
    auto *esp32_server = ConfigConsole::instance().server();
    if (esp32_server == nullptr) {
        printf("USB/IP 服务未启动\n");
        return 0;
    }
    const auto devices = esp32_server->list_device_snapshots();
    if (devices.empty()) {
        printf("当前没有已接入的 USB 设备\n");
        return 0;
    }
    // 空闲设备在前、被客户端占用的在后（见 list_device_snapshots）
    for (const auto &device: devices) {
        printf("%-8s VID:PID=%04x:%04x  %s\n", device.busid.c_str(), device.vendor_id,
               device.product_id, device.in_use ? "使用中（已被远程占用）" : "空闲（可共享）");
    }
    return 0;
}

// 打印内存占用：thread_main 里周期打的那份 heap 数据，改为按需查看
int cmd_mem(int argc, char **argv)
{
    const int nerrors = arg_parse(argc, argv, reinterpret_cast<void **>(&s_mem_args));
    if (nerrors != 0) {
        arg_print_errors(stderr, s_mem_args.end, argv[0]);
        return 1;
    }
    printf("Free: %lu, Min: %lu\n",
           static_cast<unsigned long>(esp_get_free_heap_size()),
           static_cast<unsigned long>(esp_get_minimum_free_heap_size()));
    printf("DMA free: %lu, DMA min: %lu, DMA max block: %lu\n",
           static_cast<unsigned long>(heap_caps_get_free_size(MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL)),
           static_cast<unsigned long>(heap_caps_get_minimum_free_size(MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL)),
           static_cast<unsigned long>(heap_caps_get_largest_free_block(MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL)));
    printf("PSRAM free: %lu, PSRAM min: %lu\n",
           static_cast<unsigned long>(heap_caps_get_free_size(MALLOC_CAP_SPIRAM)),
           static_cast<unsigned long>(heap_caps_get_minimum_free_size(MALLOC_CAP_SPIRAM)));
    return 0;
}

// argtable 参数声明（init 里注册命令前调用一次）
void register_console_command_argtables()
{
    s_wifi_set_args.ssid = arg_str1(nullptr, nullptr, "<ssid>", "目标 WiFi 名称");
    s_wifi_set_args.password = arg_str0(nullptr, nullptr, "[password]", "密码，开放网络省略");
    s_wifi_set_args.end = arg_end(2);

    s_wifi_show_args.end = arg_end(2);
    s_wifi_reset_args.end = arg_end(2);
    s_logs_args.end = arg_end(2);
    s_about_args.end = arg_end(2);
    s_devices_args.end = arg_end(2);
    s_mem_args.end = arg_end(2);
}

} // namespace

esp_err_t ConfigConsole::init()
{
#if !CONFIG_USBIPD_CFG_CONSOLE_ENABLE
    return ESP_OK;
#endif

    s_config_uart_port = static_cast<uart_port_t>(CONFIG_USBIPD_CFG_UART_NUM);

    // ---------- 日志镜像钩子（必须在大量日志开始前安装） ----------
    // 只在启动早期向 default logger 的 sinks 追加一次镜像 sink，运行中不增删，
    // 避免与并发打日志形成 data race（见 UartMirrorSink 类注释）
    auto logger = spdlog::default_logger();
    if (logger == nullptr) {
        // spdlog 尚未初始化（正常路径不会发生：项目代码启动即打日志），
        // 跳过 spdlog 镜像，ESP_LOG 镜像不受影响
        ESP_LOGW(TAG, "spdlog default logger 不可用，spdlog 日志不会镜像");
    }
    else {
        auto mirror_sink = std::make_shared<UartMirrorSink>(s_config_uart_port);
        // 默认 formatter 的 eol 是裸 \n，真实 UART 终端需要 \r\n 才正常回行首。
        // 主 formatter 是默认 pattern（"%+"，无人 set_pattern），这里只改 eol：
        // format 输出与主通道一致、行尾为 \r\n，整行一次写出，无需再在调用线程
        // 栈上做规范化缓冲（nio 等小栈线程会被大栈数组压爆）
        mirror_sink->set_formatter(std::make_unique<spdlog::pattern_formatter>(
                "%+", spdlog::pattern_time_type::local, "\r\n"));
        logger->sinks().push_back(mirror_sink);
    }

    s_orig_log_vprintf = esp_log_set_vprintf(&ConfigConsole::mirror_log_vprintf);

    // ---------- REPL ----------
    esp_console_repl_config_t repl_config = {};
    repl_config.max_history_len = 8;
    repl_config.max_cmdline_length = 256;
    repl_config.task_stack_size = 8192;
    repl_config.prompt = "usbipd>";

    esp_console_dev_uart_config_t uart_config = {
            .channel = CONFIG_USBIPD_CFG_UART_NUM,
            .baud_rate = 115200,
            .tx_gpio_num = CONFIG_USBIPD_CFG_UART_TX_GPIO,
            .rx_gpio_num = CONFIG_USBIPD_CFG_UART_RX_GPIO,
    };

    esp_console_repl_t *repl = nullptr;
    esp_err_t err = esp_console_new_repl_uart(&uart_config, &repl_config, &repl);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "REPL 创建失败（UART%d, tx=%d, rx=%d）: %s",
                 CONFIG_USBIPD_CFG_UART_NUM, CONFIG_USBIPD_CFG_UART_TX_GPIO,
                 CONFIG_USBIPD_CFG_UART_RX_GPIO, esp_err_to_name(err));
        return err;
    }

    // argtable 参数声明（先于命令注册；结构体生命周期与命令一致）
    register_console_command_argtables();

    const esp_console_cmd_t wifi_set_cmd = {
            .command = "wifi_set",
            .help = "设置 WiFi 配置并重连（NVS 持久化）: wifi_set <ssid> [password]，"
                    "省略 password 视为开放网络",
            .hint = nullptr, // argtable 非空时自动生成 hint
            .func = &cmd_wifi_set,
            .argtable = &s_wifi_set_args,
            .func_w_context = nullptr,
            .context = nullptr,
    };
    err = esp_console_cmd_register(&wifi_set_cmd);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "注册 wifi_set 命令失败: %s", esp_err_to_name(err));
        return err;
    }

    const esp_console_cmd_t wifi_show_cmd = {
            .command = "wifi_show",
            .help = "显示当前 WiFi 配置与连接状态",
            .hint = nullptr,
            .func = &cmd_wifi_show,
            .argtable = &s_wifi_show_args,
            .func_w_context = nullptr,
            .context = nullptr,
    };
    err = esp_console_cmd_register(&wifi_show_cmd);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "注册 wifi_show 命令失败: %s", esp_err_to_name(err));
        return err;
    }

    const esp_console_cmd_t wifi_reset_cmd = {
            .command = "wifi_reset",
            .help = "清除 NVS 里的 WiFi 配置（下次开机回编译期默认）",
            .hint = nullptr,
            .func = &cmd_wifi_reset,
            .argtable = &s_wifi_reset_args,
            .func_w_context = nullptr,
            .context = nullptr,
    };
    err = esp_console_cmd_register(&wifi_reset_cmd);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "注册 wifi_reset 命令失败: %s", esp_err_to_name(err));
        return err;
    }

    const esp_console_cmd_t logs_cmd = {
            .command = "logs",
            .help = "把 UART0 系统日志镜像到本配置口，Ctrl-C 停止",
            .hint = nullptr,
            .func = &cmd_logs,
            .argtable = &s_logs_args,
            .func_w_context = nullptr,
            .context = nullptr,
    };
    err = esp_console_cmd_register(&logs_cmd);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "注册 logs 命令失败: %s", esp_err_to_name(err));
        return err;
    }

    const esp_console_cmd_t about_cmd = {
            .command = "about",
            .help = "简介：这个固件是什么、有哪些管理接口",
            .hint = nullptr,
            .func = &cmd_about,
            .argtable = &s_about_args,
            .func_w_context = nullptr,
            .context = nullptr,
    };
    err = esp_console_cmd_register(&about_cmd);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "注册 about 命令失败: %s", esp_err_to_name(err));
        return err;
    }

    const esp_console_cmd_t devices_cmd = {
            .command = "devices",
            .help = "列出已接入的 USB 设备（busid/VID:PID/占用状态）",
            .hint = nullptr,
            .func = &cmd_devices,
            .argtable = &s_devices_args,
            .func_w_context = nullptr,
            .context = nullptr,
    };
    err = esp_console_cmd_register(&devices_cmd);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "注册 devices 命令失败: %s", esp_err_to_name(err));
        return err;
    }

    const esp_console_cmd_t mem_cmd = {
            .command = "mem",
            .help = "打印当前内存使用情况",
            .hint = nullptr,
            .func = &cmd_mem,
            .argtable = &s_mem_args,
            .func_w_context = nullptr,
            .context = nullptr,
    };
    err = esp_console_cmd_register(&mem_cmd);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "注册 mem 命令失败: %s", esp_err_to_name(err));
        return err;
    }

    err = esp_console_start_repl(repl);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "REPL 启动失败: %s", esp_err_to_name(err));
        return err;
    }

    ESP_LOGI(TAG, "配置口 console 已启动: UART%d tx=GPIO%d rx=GPIO%d",
             CONFIG_USBIPD_CFG_UART_NUM, CONFIG_USBIPD_CFG_UART_TX_GPIO,
             CONFIG_USBIPD_CFG_UART_RX_GPIO);
    return ESP_OK;
}
