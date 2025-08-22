#include <iostream>
#include <thread>
#include <semaphore>

#include <nvs_flash.h>
#include <esp_log.h>
#include <esp_system.h>
#include <esp_wifi.h>
#include <esp_pthread.h>
#include <usb/usb_host.h>

#include <lwip/sys.h>
#include <lwip/sockets.h>

#include <asio.hpp>
#include <spdlog/spdlog.h>

#include <pthread.h>

#include "esp32_handler/Esp32Server.h"
#include "mock_mouse.h"


using namespace std;

auto TAG = "tcpip_test";

std::atomic_bool wifi_thread_should_stop = false;

std::binary_semaphore wifi_reconnect_semaphore{0};
std::thread wifi_connect_thread;

auto wifi_ssid = "your ssid";
auto wifi_passwd = "your passwd";

constexpr std::uint16_t listening_port = 3240;

esp_pthread_cfg_t create_config(const char *name, int core_id, int stack, int prio) {
    auto cfg = esp_pthread_get_default_config();
    cfg.thread_name = name;
    cfg.pin_to_core = core_id;
    cfg.stack_size = stack;
    cfg.prio = prio;
    return cfg;
}

static void event_handler(void *arg, esp_event_base_t event_base,
                          int32_t event_id, void *event_data) {
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    }
    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        ESP_LOGI(TAG, "connect to the AP fail");
        wifi_reconnect_semaphore.release();
    }
    else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        auto *event = static_cast<ip_event_got_ip_t *>(event_data);
        ESP_LOGI(TAG, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
    }
}


void start_always_try_connecting_to_wifi() {
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
        nullptr,
        &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
        IP_EVENT_STA_GOT_IP,
        &event_handler,
        nullptr,
        &instance_got_ip));

    wifi_config_t wifi_config{};
    strncpy(reinterpret_cast<char *>(wifi_config.sta.ssid), wifi_ssid, std::size(wifi_config.sta.ssid));
    strncpy(reinterpret_cast<char *>(wifi_config.sta.password), wifi_passwd, std::size(wifi_config.sta.password));
    wifi_config.sta.scan_method = WIFI_ALL_CHANNEL_SCAN;
    wifi_config.sta.sort_method = WIFI_CONNECT_AP_BY_SIGNAL;

    esp_pthread_cfg_t pthread_cfg = esp_pthread_get_default_config();
    pthread_cfg.prio = 10;
    pthread_cfg.pin_to_core = 1; // 设置核心1
    pthread_cfg.thread_name = "wifi_connect_thread";
    esp_pthread_set_cfg(&pthread_cfg);
    wifi_connect_thread = std::thread([]() {
        while (!wifi_thread_should_stop) {
            wifi_reconnect_semaphore.acquire();
            if (wifi_thread_should_stop)
                break;
            ESP_LOGI(TAG, "wifi reconnecting");
            esp_wifi_connect();
        }
    });
    esp_pthread_cfg_t default_cfg = esp_pthread_get_default_config();
    esp_pthread_set_cfg(&default_cfg);


    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "wifi_init_sta finished.");
}

std::thread usb_host_event_thread;

void init_usb_host() {
    ESP_LOGI(TAG, "Installing USB Host Library");
    usb_host_config_t host_config = {
            .skip_phy_setup = false,
            .intr_flags = ESP_INTR_FLAG_LEVEL1,
            .enum_filter_cb = nullptr,
    };
    ESP_ERROR_CHECK(usb_host_install(&host_config));


    esp_pthread_cfg_t cfg = esp_pthread_get_default_config();
    cfg.prio = 10;
    cfg.pin_to_core = 1; // 设置核心1
    cfg.thread_name = "usb_host_event_thread";
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

    start_always_try_connecting_to_wifi();

    init_usb_host();
}

asio::awaitable<void> handle_connection(asio::ip::tcp::socket &&socket) {
    while (true) {
        try {
            std::array<char, 1> buffer{};
            co_await asio::async_read(socket, asio::buffer(buffer), asio::use_awaitable);
            co_await asio::async_write(socket, asio::buffer(buffer), asio::use_awaitable);
        } catch (std::exception &e) {
            ESP_LOGE(TAG, "socket exception occurs: %s", e.what());
            break;
        }
    }
    std::error_code ignore_ec;
    ESP_LOGI(TAG, "尝试关闭socket");
    socket.shutdown(asio::ip::tcp::socket::shutdown_both, ignore_ec);
    socket.close(ignore_ec);

    co_return;
}

using namespace usbipdcpp;

int thread_main() {
    ESP_LOGI(TAG, "初始化所有设备");
    init_all();

    spdlog::set_level(spdlog::level::trace);

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
    server.start(endpoint);

    // SPDLOG_INFO("Start turning over left button");
    // while (true) {
    //     {
    //         std::lock_guard lock(mouse_interface_handler.data_mutex);
    //         mouse_interface_handler.left_pressed = !mouse_interface_handler.left_pressed;
    //     }
    //     SPDLOG_INFO("Turn over left button");
    //     std::this_thread::sleep_for(std::chrono::seconds(1));
    // }
    while (true) {
        std::this_thread::sleep_for(chrono::seconds(3600));
    }

    server.stop();
    return 0;
}


extern "C" void app_main(void) {
    std::thread main_thread([&]() {
        ESP_LOGI(TAG, "启动主线程main函数");
        thread_main();
    });
    main_thread.join();
}
