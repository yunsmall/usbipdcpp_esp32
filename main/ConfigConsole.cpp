#include "ConfigConsole.h"

#include <atomic>
#include <cstdio>
#include <cstring>
#include <mutex>
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
#include "WifiConnection.h"
#include "sdkconfig.h"

// 配置口和系统日志口是同一个 UART 的话两个功能会抢同一个串口（Kconfig 的 help
// 只有文字提醒，这里升级成编译期硬检查）。console 不是 UART（如 USB-Serial-JTAG）
// 时 CONFIG_ESP_CONSOLE_UART_NUM 未定义或为负，不会误报
#if CONFIG_USBIPD_CFG_CONSOLE_ENABLE && defined(CONFIG_ESP_CONSOLE_UART_NUM) && \
        CONFIG_ESP_CONSOLE_UART_NUM >= 0 && \
        CONFIG_ESP_CONSOLE_UART_NUM == CONFIG_USBIPD_CFG_UART_NUM
#error "USBIPD_CFG_UART_NUM 与 CONFIG_ESP_CONSOLE_UART_NUM 相同：配置口会和系统日志抢同一个串口"
#endif

namespace
{

constexpr const char *TAG = "ConfigConsole";

// 日志镜像总开关：esp_log hook 与 spdlog sink 都检查它，logs 命令翻转。
// relaxed 是刻意的：它只决定"这一行要不要镜像"，不承担同步职责——读到旧值
// 最多让切换瞬间多/少镜像一行，没有别的数据依赖它
std::atomic<bool> s_log_mirror_enabled{false};

// 配置口写入互斥：两条镜像路径（esp_log hook、spdlog sink）来自不同线程、
// 写的是同一个 UART，而 uart_write_bytes 的互斥只在单次调用内部——不串行化的
// 话，补 \r 拆出的多段写会和另一线程的日志交错成半行。锁只在镜像开启时才被
// 碰到，代价可忽略。
// 前提与本文件其它日志处理一致：日志都在任务上下文——ESP_LOGx 本来就禁止在
// ISR 里调用（ISR 要打日志得走 ESP_EARLY_LOGx，那条路径直接走 ROM 输出，
// 不经过 esp_log_set_vprintf 装的 hook），所以这里拿锁是安全的
std::mutex s_mirror_write_mutex;

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
        std::lock_guard lock(s_mirror_write_mutex);
        uart_write_bytes(port_, formatted.data(), formatted.size());
    }

    void flush_() override {}

private:
    uart_port_t port_;
};

// 配置口 UART 号（Kconfig 编译期常量；REPL、镜像钩子、日志镜像命令共用这一份）
constexpr uart_port_t s_config_uart_port = static_cast<uart_port_t>(CONFIG_USBIPD_CFG_UART_NUM);

} // anonymous namespace

// 与头文件 static 成员定义一一对应
std::atomic<vprintf_like_t> ConfigConsole::s_orig_log_vprintf{nullptr};

ConfigConsole &ConfigConsole::instance()
{
    static ConfigConsole console;
    return console;
}

int ConfigConsole::mirror_log_vprintf(const char *fmt, va_list args)
{
    // UART0 输出保持原样：调 esp_log_set_vprintf 返回的原实现。
    // 只 va_end 自己 va_copy 出来的复制品：args 是调用方 va_start 出来的，
    // 归调用方 va_end（对同一条 va_list 二次 end 是未定义行为）
    int ret = 0;
    const vprintf_like_t orig = s_orig_log_vprintf.load();
    if (orig != nullptr) {
        va_list args_uart0;
        va_copy(args_uart0, args);
        ret = orig(fmt, args_uart0);
        va_end(args_uart0);
    }
    else {
        // 装钩子和保存旧实现之间有个极短窗口（见 init 里那行赋值的注释），
        // 此刻读到的还是 nullptr。就这么跳过的话，这几行日志会从 UART0 上
        // 彻底消失——退回 libc 的 stdout 兜底：IDF 的 console 本来就接在
        // 那儿，目的地是同一个。REPL 是在 init 的更靠后处才启动的，窗口期
        // stdout 还接在 UART0 上，不存在"兜底把日志写去配置口"的情况
        va_list args_uart0;
        va_copy(args_uart0, args);
        ret = vprintf(fmt, args_uart0);
        va_end(args_uart0);
    }

    if (s_log_mirror_enabled.load(std::memory_order_relaxed)) {
        va_list args_mirror;
        va_copy(args_mirror, args);
        // 缓冲不加大是刻意的：本函数跑在打日志那条线程自己的栈上，512B 数组曾把
        // nio 这类小栈线程压爆（见 uart_write_normalized 注释），超长行只能截断
        char buf[256];
        const int len = std::vsnprintf(buf, sizeof(buf), fmt, args_mirror);
        va_end(args_mirror);
        if (len > 0) {
            std::size_t to_write = static_cast<std::size_t>(len) < sizeof(buf)
                                           ? static_cast<std::size_t>(len)
                                           : sizeof(buf) - 1;
            if (static_cast<std::size_t>(len) >= sizeof(buf)) {
                // 超长行被截断：尾部换成 "..." + 换行——既提示"这行还有内容没显
                // 示"，也把行收干净（残行不带换行符的话，下一条日志会接在它后面，
                // 看着像丢了行）
                // 落点按"写入上界（buf-1）往前退 tail_len"算：拷贝范围与写入
                // 范围严格错开，不碰 buf 末尾的终止符
                static constexpr char tail[] = "...\n";
                constexpr std::size_t tail_len = sizeof(tail) - 1; // 不含终止符
                std::memcpy(buf + (sizeof(buf) - 1) - tail_len, tail, tail_len);
                to_write = sizeof(buf) - 1;
            }
            // 与 spdlog 那条镜像路径以及其它线程的 ESP_LOG 互斥：本函数可能把
            // 一行拆成多段写（补 \r），不串行化会在镜像口交错成半行
            std::lock_guard lock(s_mirror_write_mutex);
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
    return esp32_server_.load();
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

// ap_set：配网热点的名称与密码（密码可选，缺省 = 开放热点）
struct ApSetArgs
{
    struct arg_str *ssid;
    struct arg_str *password;
    struct arg_end *end;
};

ApSetArgs s_ap_set_args = {};

// wifi_mode：期望的工作模式（可选参数，省略 = 只看不改）
struct WifiModeArgs
{
    struct arg_str *mode;
    struct arg_end *end;
};

WifiModeArgs s_wifi_mode_args = {};

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

// arg_parse 把"结构体地址"当 void*[] 遍历（IDF 示例 .argtable = &结构体 的传统
// 写法）：前提是成员按声明顺序紧排、无填充。下面把这个前提变成编译期检查——成员
// 全是同尺寸的指针，sizeof 等于成员数×sizeof(void*) 就说明没有填充；哪天不成立
// （换编译器、给结构体加了别的成员）编译直接失败，不会运行时读错参数表
static_assert(sizeof(WifiSetArgs) == 3 * sizeof(void *));
static_assert(sizeof(ApSetArgs) == 3 * sizeof(void *));
static_assert(sizeof(WifiModeArgs) == 2 * sizeof(void *));
static_assert(sizeof(WifiShowArgs) == 1 * sizeof(void *));

int cmd_wifi_set(int argc, char **argv)
{
    const int nerrors = arg_parse(argc, argv, reinterpret_cast<void **>(&s_wifi_set_args));
    if (nerrors != 0) {
        arg_print_errors(stderr, s_wifi_set_args.end, argv[0]);
        // 用法串里的 <> [] 是占位记法，补个真实示例免得被照抄输入
        printf("例: wifi_set MyAP 12345678（尖括号/方括号只是占位记号，输入时不带）\n");
        return 1;
    }
    // 密码含空格时用双引号包裹或反斜杠转义输入（esp_console 分词器支持，
    // 与 bash 规则一致）
    const char *password = s_wifi_set_args.password->count > 0 ? s_wifi_set_args.password->sval[0] : "";
    const char *ssid = s_wifi_set_args.ssid->sval[0];

    // apply_config 要等到实连结果（最长十几秒）才返回，先给提示免得像卡死
    printf("正在连接 \"%s\"（最多等 %d 秒）...\n", ssid, WifiConfigManager::APPLY_TIMEOUT_SECONDS);
    bool persisted = false;
    esp_err_t err = WifiConfigManager::instance().apply_config(ssid, password, &persisted);
    if (err == ESP_OK) {
        // NVS 写失败时 apply_config 仍按成功返回（连接已切过去）：提示如实分岔，
        // 别把"重启后回退"说成"已保存"
        if (persisted) {
            printf("\n连接成功，配置已保存（重启后仍生效）\n");
        }
        else {
            printf("\n连接成功，但配置没能写入 NVS（重启后会回退到原配置）\n");
        }
        return 0;
    }
    // 失败：先一行说明原因，再一行说明配置的去向
    if (err == ESP_ERR_TIMEOUT) {
        printf("\n连接超时：%d 秒内没连上\n", WifiConfigManager::APPLY_TIMEOUT_SECONDS);
    }
    else if (err == ESP_FAIL) {
        printf("\n连接失败：密码错误或找不到该 AP\n");
    }
    else {
        printf("\n设置失败: %s\n", esp_err_to_name(err));
    }
    printf("配置未保存，继续使用原配置\n");
    return 1;
}

// ap_set：设置配网热点的名称/密码（存 NVS，下次热点启动时生效）
int cmd_ap_set(int argc, char **argv)
{
    const int nerrors = arg_parse(argc, argv, reinterpret_cast<void **>(&s_ap_set_args));
    if (nerrors != 0) {
        arg_print_errors(stderr, s_ap_set_args.end, argv[0]);
        printf("例: ap_set usbipd-setup 12345678（密码留空 = 开放热点）\n");
        return 1;
    }
    const char *password = s_ap_set_args.password->count > 0 ? s_ap_set_args.password->sval[0] : "";
    const char *ssid = s_ap_set_args.ssid->sval[0];
    esp_err_t err = WifiConfigManager::instance().apply_ap_config(ssid, password);
    if (err != ESP_OK) {
        printf("\n设置失败: %s\n", esp_err_to_name(err));
        printf("热点密码要么留空（开放），要么 8~63 位\n");
        return 1;
    }
    printf("\n配网热点配置已保存：\"%s\"（%s）\n", ssid,
           password[0] == '\0' ? "开放网络" : "WPA2 加密");
    if (WifiConnection::instance().is_ap_active()) {
        printf("当前热点仍用旧配置，下次开热点时生效\n");
    }
    else {
        printf("等设备连不上 WiFi 时会自动用它开热点，连上后访问 http://192.168.4.1/ 配网\n");
    }
    return 0;
}

// wifi_mode：查看/设置期望的工作模式（存 NVS，重启后沿用）
int cmd_wifi_mode(int argc, char **argv)
{
    const int nerrors = arg_parse(argc, argv, reinterpret_cast<void **>(&s_wifi_mode_args));
    if (nerrors != 0) {
        arg_print_errors(stderr, s_wifi_mode_args.end, argv[0]);
        printf("例: wifi_mode ap（只做配网热点）/ wifi_mode sta（连接 WiFi）\n");
        return 1;
    }
    auto &manager = WifiConfigManager::instance();
    if (s_wifi_mode_args.mode->count == 0) {
        if (manager.work_mode() == WifiConfigManager::WifiWorkMode::Ap) {
            printf("当前工作模式: ap（只做配网热点，不连 WiFi）\n");
        }
        else {
            printf("当前工作模式: sta（连 WiFi；连不上 %d 秒后自动开配网热点）\n",
                   WifiConfigManager::AP_FALLBACK_SECONDS);
        }
        return 0;
    }

    const std::string mode_str = s_wifi_mode_args.mode->sval[0];
    WifiConfigManager::WifiWorkMode mode;
    if (!WifiConfigManager::parse_work_mode(mode_str, mode)) {
        printf("未知模式 \"%s\"：只接受 sta 或 ap\n", mode_str.c_str());
        return 1;
    }

    const esp_err_t err = manager.set_work_mode(mode);
    if (err != ESP_OK) {
        printf("设置失败: %s\n", esp_err_to_name(err));
        return 1;
    }
    if (mode == WifiConfigManager::WifiWorkMode::Ap) {
        printf("已切到 AP 模式（存 NVS，重启后沿用）：不再尝试连 WiFi，配网热点随即开启\n");
    }
    else {
        printf("已切到 STA 模式（存 NVS，重启后沿用）：正在尝试连接 \"%s\"\n",
               manager.ssid().c_str());
    }
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
    printf("工作模式: %s\n", manager.work_mode() == WifiConfigManager::WifiWorkMode::Ap
                                    ? "ap（只做配网热点，不连 WiFi）"
                                    : "sta（连接 WiFi）");
    printf("当前配置 SSID: %s\n", ssid.c_str());
    printf("密码: %s\n", manager.password().empty() ? "<空（开放网络）>" : "********");
    printf("连接状态: %s\n", manager.is_connected() ? "已连接" : "未连接");
    printf("IP: %s\n", ip.empty() ? "<无>" : ip.c_str());
    // 配网热点：连不上时用户最需要知道的两件事——热点叫什么、开没开
    printf("配网热点: %s（%s）\n", manager.ap_ssid().c_str(),
           WifiConnection::instance().is_ap_active() ? "运行中，浏览器访问 http://192.168.4.1/"
                                                     : "未启动，连不上 WiFi 时自动开");
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
    printf("已清除 NVS 记录（含配网热点配置），内存配置回退编译期默认，当前连接不变\n");
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
    // 因此本命令自行轮询配置口 RX 收 0x03 作为退出信号。
    // 依赖 esp_console 当前的行为（命令在 REPL 任务里同步执行、期间不读 RX）；
    // 若哪天 REPL 改成边执行边收输入，这里就会与它抢 RX，得改成事件通知退出
    // flush 清的是上次"退出时连按 Ctrl-C"留下的残余 0x03：不清的话进入本命令
    // 会立刻读到它而秒退。回车后马上又按的 Ctrl-C 也可能被这次清掉——缓冲里
    // 没有时间戳分不出两者，只能取更常见的"连按遗留"优先
    uart_flush_input(s_config_uart_port);
    // 提示先于开关：printf 直接写配置口、不走镜像那把锁（s_mirror_write_mutex），
    // 先开镜像的话这行提示可能和别的线程的日志交错成半行
    printf("日志镜像已开启（UART0 日志同时显示在此口），按 Ctrl-C 停止\n");
    ConfigConsole::set_log_mirror(true);

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
    {
        // set_log_mirror(false) 只挡得住后来的调用：已经进了临界区的那行日志
        // 可能还在写同一根 UART。这行提示进同一把锁，免得两段字节交错
        std::lock_guard lock(s_mirror_write_mutex);
        printf("\n日志镜像已停止\n");
    }
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
            "  ap_set          设置配网热点名称/密码（连不上 WiFi 时自动开热点）\n"
            "  wifi_mode       查看/设置工作模式：sta=连 WiFi，ap=只做配网热点\n"
            "  logs            实时查看主串口日志，Ctrl-C 退出\n"
            "联网后也可用浏览器打开 http://<本机IP>/ 页面配网；\n"
            "连不上 WiFi 时设备自己开热点，手机/电脑连上后打开 http://192.168.4.1/ 配网\n");
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
        printf("%-8s VID:PID=%04x:%04x  %s\n", device.busid.c_str(),
               static_cast<unsigned>(device.vendor_id),
               static_cast<unsigned>(device.product_id),
               device.in_use ? "使用中（已被远程占用）" : "空闲（可共享）");
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

// argtable 参数声明（init 里注册命令前调用一次）。
// arg_end(n) 的 n 是"最多显示几条错误"的槽位容量，与校验松紧无关：多余参数
// 无论 n 取几都一律报错（arg_parse_untagged 会把每个未匹配 token 都注册成错误，
// 实测 `wifi_set a b c` 即 nerrors=1），n=2 只是多留一条错误的显示位
void register_console_command_argtables()
{
    s_wifi_set_args.ssid = arg_str1(nullptr, nullptr, "<ssid>", "目标 WiFi 名称");
    s_wifi_set_args.password = arg_str0(nullptr, nullptr, "[password]", "密码，开放网络省略");
    s_wifi_set_args.end = arg_end(2);

    s_ap_set_args.ssid = arg_str1(nullptr, nullptr, "<ssid>", "配网热点名称");
    s_ap_set_args.password = arg_str0(nullptr, nullptr, "[password]", "密码（至少 8 位），开放热点省略");
    s_ap_set_args.end = arg_end(2);

    s_wifi_mode_args.mode = arg_str0(nullptr, nullptr, "[sta|ap]",
                                     "工作模式：sta=连 WiFi，ap=只做配网热点（省略则显示当前模式）");
    s_wifi_mode_args.end = arg_end(2);

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
    // 幂等：重复执行会把上一轮装好的镜像钩子当成"原 vprintf"再存一次，
    // mirror_log_vprintf 于是调到自己——一打日志就无限递归、栈溢出；镜像 sink
    // 也会被重复挂进 logger。首次结果存下来，重复调用直接返回它。
    // 失败也不重试（call_once 的语义）：REPL 起不来的原因（UART 号/引脚冲突）
    // 在 Kconfig 里，运行中重试没有意义，所以刻意不留重试路径
    static std::once_flag init_once;
    static esp_err_t init_result = ESP_OK;
    std::call_once(init_once, [] { init_result = init_repl_and_mirror(); });
    return init_result;
}

esp_err_t ConfigConsole::init_repl_and_mirror()
{
    // ---------- 日志镜像钩子（必须在大量日志开始前安装） ----------
    // 只在启动早期向 default logger 的 sinks 追加一次镜像 sink，运行中不增删，
    // 避免与并发打日志形成 data race（见 UartMirrorSink 类注释）
    auto logger = spdlog::default_logger();
    if (logger == nullptr) {
        // 防御分支：default_logger() 由 spdlog registry 构造时建好，正常不为空
        // （除非启用编译宏 SPDLOG_DISABLE_DEFAULT_LOGGER，或有人显式
        // set_default_logger(nullptr)——本项目两者都没有）；真为空也只是少这条
        // 镜像通道，ESP_LOG 镜像不受影响
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

    // C++17 起赋值先算右边：esp_log_set_vprintf 当场把钩子装上，之后才轮到把
    // 旧实现存进来。这中间若有别处打日志，钩子读到的还是 nullptr——那里用
    // stdout 兜底（见 mirror_log_vprintf），不会丢行
    s_orig_log_vprintf.store(esp_log_set_vprintf(&ConfigConsole::mirror_log_vprintf));

    // ---------- REPL ----------
    // 命令里的 printf 会写到配置口、不是 UART0：REPL 任务启动时发现本口不是
    // CONFIG_ESP_CONSOLE_UART_NUM，会把自己任务的 stdin/stdout/stderr 重定向到
    // /dev/uart/<本口>（IDF console 组件 esp_console_common.c 的 esp_console_repl_task），
    // 而命令正是在那个任务里执行的；ESP-IDF 的 newlib 里这三个流按任务记录，
    // 其它任务（UART0 日志等）不受影响
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
        // 上面装好的镜像钩子故意不撤：镜像开关默认是关的，REPL 没起来就没人能
        // 打开它（logs 命令不存在）；撤销反而要在"可能正被并发打日志"的路径上
        // 动 sinks()，风险比收益大
        return err;
    }

    // 从这里起的失败路径都不回收 repl 句柄：IDF 的清理入口 repl->del（即
    // esp_console_stop_repl）要求 REPL 已经 start 过——未启动时它内部会因
    // s_interrupt_reading_fd 还没就绪提前返回，照样不释放（见 esp_console_repl_
    // internal.c 的 esp_console_common_deinit）。加上本函数被 call_once 保护、
    // 只执行一次，走到这里失败设备本来就该重启，不值得为它写一段不生效的伪清理
    // argtable 参数声明（先于命令注册；结构体生命周期与命令一致）
    register_console_command_argtables();

    // help 是 esp_console 列表里的一行，保持简短；真实示例放在参数报错时打印
    const esp_console_cmd_t wifi_set_cmd = {
            .command = "wifi_set",
            .help = "设置 WiFi 并重连；连上 AP 才保存进 NVS，失败不保存。省略 password 即开放网络",
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

    const esp_console_cmd_t ap_set_cmd = {
            .command = "ap_set",
            .help = "设置配网热点的名称/密码（连不上 WiFi 时自动开启，存 NVS）",
            .hint = nullptr,
            .func = &cmd_ap_set,
            .argtable = &s_ap_set_args,
            .func_w_context = nullptr,
            .context = nullptr,
    };
    err = esp_console_cmd_register(&ap_set_cmd);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "注册 ap_set 命令失败: %s", esp_err_to_name(err));
        return err;
    }

    const esp_console_cmd_t wifi_mode_cmd = {
            .command = "wifi_mode",
            .help = "查看/设置工作模式：sta=连 WiFi，ap=只做配网热点（存 NVS）",
            .hint = nullptr,
            .func = &cmd_wifi_mode,
            .argtable = &s_wifi_mode_args,
            .func_w_context = nullptr,
            .context = nullptr,
    };
    err = esp_console_cmd_register(&wifi_mode_cmd);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "注册 wifi_mode 命令失败: %s", esp_err_to_name(err));
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
            .help = "清除 NVS 里的 WiFi/配网热点配置（下次开机回编译期默认）",
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
