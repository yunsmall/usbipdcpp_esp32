#include "sdkconfig.h"

#include <cstring>
#include <iostream>
#include <thread>

#include <nvs_flash.h>
#include <esp_log.h>
#include <esp_system.h>
#include <esp_pthread.h>
#include <usb/usb_host.h>

#include <lwip/sys.h>
#include <lwip/sockets.h>

#include <asio.hpp>
#include <spdlog/spdlog.h>

#include <pthread.h>

#include "esp32_handler/Esp32Server.h"

#include "ConfigConsole.h"
#include "HttpConfigApi.h"
#include "WifiConfigManager.h"
#include "WifiConnection.h"

using namespace std;

auto TAG = "tcpip_test";

constexpr std::uint16_t listening_port = 3240;

esp_pthread_cfg_t create_config(const char *name, int core_id, int stack, int prio) {
    auto cfg = esp_pthread_get_default_config();
    cfg.thread_name = name;
    cfg.pin_to_core = core_id;
    cfg.stack_size = stack;
    cfg.prio = prio;
    return cfg;
}

std::thread usb_host_event_thread;

void init_usb_host() {
    ESP_LOGI(TAG, "Installing USB Host Library");
    usb_host_config_t host_config = {
            .skip_phy_setup = false,
            .intr_flags = ESP_INTR_FLAG_LEVEL3,
            .enum_filter_cb = nullptr,
    };
    ESP_ERROR_CHECK(usb_host_install(&host_config));


    esp_pthread_cfg_t cfg = esp_pthread_get_default_config();
    cfg.prio = 10;
    cfg.pin_to_core = 1; // 设置核心1
    cfg.thread_name = "usb_host_event_thread";
    cfg.stack_size = 4096;
    esp_pthread_set_cfg(&cfg);

    usb_host_event_thread = std::thread([]() {
        bool has_clients = true;
        bool has_devices = false;
        while (has_clients) {
            uint32_t event_flags;
            ESP_ERROR_CHECK(usb_host_lib_handle_events(portMAX_DELAY, &event_flags));
            if (event_flags & USB_HOST_LIB_EVENT_FLAGS_NO_CLIENTS) {
                ESP_LOGI(TAG, "Get FLAGS_NO_CLIENTS");
                if (ESP_OK == usb_host_device_free_all()) {
                    ESP_LOGI(TAG, "All devices marked as free, no need to wait FLAGS_ALL_FREE event");
                    has_clients = false;
                }
                else {
                    ESP_LOGI(TAG, "Wait for the FLAGS_ALL_FREE");
                    has_devices = true;
                }
            }
            if (has_devices && event_flags & USB_HOST_LIB_EVENT_FLAGS_ALL_FREE) {
                ESP_LOGI(TAG, "Get FLAGS_ALL_FREE");
                has_clients = false;
            }
        }
        ESP_LOGI(TAG, "No more clients and devices, uninstall USB Host library");

        //Uninstall the USB Host Library
        ESP_ERROR_CHECK(usb_host_uninstall());
    });
    esp_pthread_cfg_t default_cfg = esp_pthread_get_default_config();
    esp_pthread_set_cfg(&default_cfg);
}

void init_all() {
    //Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    ESP_LOGI(TAG, "初始化nvs结束");

    if (CONFIG_LOG_MAXIMUM_LEVEL > CONFIG_LOG_DEFAULT_LEVEL) {
        /* If you only want to open more logs in the wifi module, you need to make the max level greater than the default level,
         * and call esp_log_level_set() before esp_wifi_init() to improve the log level of the wifi module. */
        esp_log_level_set("wifi", static_cast<esp_log_level_t>(CONFIG_LOG_MAXIMUM_LEVEL));
    }

    ESP_LOGI(TAG, "ESP_WIFI_MODE_STA");

    // WiFi 初始化 + 自动连接/重连（实现已拆到 WifiConnection）
    WifiConnection::instance().start();

    init_usb_host();
}

using namespace usbipdcpp;

int thread_main() {
    // 配置口 console 必须先于一切日志初始化：镜像钩子（esp_log hook + spdlog sink
    // 追加）要在任何并发打日志之前装好（sinks() 裸引用 vector 运行中增删有 data
    // race，见 ConfigConsole.cpp UartMirrorSink 注释）。REPL 命令的 wifi set 真正
    // 执行发生在用户输入时，彼时 init_all 早已完成，顺序上无依赖
    ConfigConsole::instance().init();

    // 刷机后 WiFi 连不上时配置口串口是唯一救急入口，开机日志直接给出接线
    // 信息——引脚随板子/Kconfig 变化，只写在 README 里刷完机根本找不到
#if CONFIG_USBIPD_CFG_CONSOLE_ENABLE
    ESP_LOGI(TAG, "WiFi 连不上时用配置口串口配网: UART%d TX=GPIO%d RX=GPIO%d @115200 8N1，命令 wifi_set <ssid> [password]",
             CONFIG_USBIPD_CFG_UART_NUM, CONFIG_USBIPD_CFG_UART_TX_GPIO,
             CONFIG_USBIPD_CFG_UART_RX_GPIO);
#endif

    ESP_LOGI(TAG, "初始化所有设备");
    init_all();

    ESP_LOGI(TAG, "连接wifi ssid:%s", WifiConfigManager::instance().ssid().c_str());
    // The password is intentionally not logged — serial logs are often shared
    // (in issue reports, screen shares, crash dumps) and this would leak the
    // WiFi credential. If you need to verify the configured password for
    // debugging, read CONFIG_USBIPD_WIFI_PASSWORD from sdkconfig directly.
    ESP_LOGI(TAG, "连接wifi password: <redacted, length=%d>", (int)WifiConfigManager::instance().password().size());

    spdlog::set_level(spdlog::level::trace);

    // HTTP 配置服务（绑定 0.0.0.0，WiFi 断/连不影响监听）
    HttpConfigApi::instance().init();

    asio::ip::tcp::endpoint listen_endpoint(asio::ip::tcp::v4(), listening_port);

    // StringPool string_pool;
    //
    // std::vector<UsbInterface> interfaces = {
    //         UsbInterface{
    //                 .interface_class = static_cast<std::uint8_t>(
    //                     ClassCode::HID),
    //                 .interface_subclass = 0x00,
    //                 .interface_protocol = 0x00,
    //                 .endpoints = {
    //                         UsbEndpoint{
    //                                 .address = 0x81, // IN
    //                                 .attributes = 0x03,
    //                                 // 8 bytes
    //                                 .max_packet_size = 8,
    //                                 // Interrupt
    //                                 .interval = 10
    //                         }
    //                 },
    //                 .handler = {},
    //         }
    // };
    // auto &mouse_interface_handler = *interfaces[0].with_handler<MockMouseInterfaceHandler>(string_pool);
    //
    //
    // auto mock_mouse = std::make_shared<UsbDevice>(UsbDevice{
    //         .path = "/usbipdcpp/mock_mouse",
    //         .busid = "2-1",
    //         .bus_num = 2,
    //         .dev_num = 1,
    //         .speed = static_cast<std::uint32_t>(UsbSpeed::Low),
    //         .vendor_id = 0x1234,
    //         .product_id = 0x5678,
    //         .device_bcd = 0xabcd,
    //         .device_class = 0x00,
    //         .device_subclass = 0x00,
    //         .device_protocol = 0x00,
    //         .configuration_value = 1,
    //         .num_configurations = 1,
    //         .interfaces = interfaces,
    //         .ep0_in = UsbEndpoint::get_default_ep0_in(),
    //         .ep0_out = UsbEndpoint::get_default_ep0_out(),
    //         .handler = {},
    // });
    // mock_mouse->with_handler<SimpleVirtualDeviceHandler>(string_pool);

    Esp32Server server;
    server.init_client();
    // server.add_device(std::move(mock_mouse));

    asio::ip::tcp::endpoint endpoint{asio::ip::tcp::v4(), listening_port};
    auto ec = server.start(endpoint);
    if (ec) [[unlikely]] {
        ESP_LOGE(TAG, "服务器启动失败：{}", ec.message());
        return -1;
    }

    // 设备面板数据源（网页 /api/devices 与配置口 devices 命令）：server 在本
    // 线程栈上、进程存活期有效，start 前访问只会拿到空列表，无害
    HttpConfigApi::instance().set_server(&server);
    ConfigConsole::instance().set_server(&server);
    
    while (true) {
        std::this_thread::sleep_for(chrono::seconds(5));
        ESP_LOGI(TAG, "Free: %lu, Min: %lu, DMA free: %lu, DMA min: %lu, PSRAM free: %lu, PSRAM min: %lu, DMA max block: %lu",
                 esp_get_free_heap_size(),
                 esp_get_minimum_free_heap_size(),
                 heap_caps_get_free_size(MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL),
                 heap_caps_get_minimum_free_size(MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL),
                 heap_caps_get_free_size(MALLOC_CAP_SPIRAM),
                 heap_caps_get_minimum_free_size(MALLOC_CAP_SPIRAM),
                 heap_caps_get_largest_free_block(MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL));
    }

    server.stop();
    return 0;
}


extern "C" void app_main(void) {
    // 必须把主流程包进 std::thread（esp_pthread → FreeRTOS 任务）再 join：
    // spdlog 的并发安全依赖 pthread 原语（mutex/guard/once），而 pthread 环境只对
    // pthread_create 创建的任务完整注册——app_main 的 main task 不是 pthread 创建的，
    // 直接在 main task 里跑 thread_main（首次打 spdlog 日志即初始化并发原语）会崩
    // （实测）。join 让 main task 保持存活，app_main 不返回
    std::thread main_thread([&]() {
        ESP_LOGI(TAG, "启动主线程main函数");
        thread_main();
    });
    main_thread.join();
}
