#pragma once

#include <usb/usb_host.h>

#include <cstdint>
#include <shared_mutex>
#include <map>
#include <mutex>
#include <string>
#include <vector>

#include <asio.hpp>


#include "usbipdcpp/Server.h"

namespace usbipdcpp
{
    class Esp32Server
    {
    public:
        Esp32Server();

        void init_client();
        // 绑定设备到可用设备列表。失败时已回滚已声明接口并关闭设备句柄，
        // 返回错误码供调用方从 host_devices 中移除该句柄
        esp_err_t bind_host_device(usb_device_handle_t dev);
        void unbind_host_device(usb_device_handle_t device);
        // start 承诺不抛异常，错误通过 error_code 报告（与 Server::start /
        // LibusbServer::start 一致，便于无异常环境的嵌入式平台）
        usbipdcpp::error_code start(asio::ip::tcp::endpoint& ep);
        void stop();

        ~Esp32Server();

        // ========== 状态查询（供网页/console 等只读展示面板用） ==========

        /**
         * @brief 单台已接入设备的只读快照
         */
        struct DeviceSnapshot
        {
            std::string busid;        // 端口拓扑 busid，客户端 attach 用这个
            std::uint16_t vendor_id = 0;
            std::uint16_t product_id = 0;
            bool in_use = false;      // true = 已被某远程客户端 attach（正在使用）
        };

        /**
         * @brief 所有已接入设备的快照（内部持 devices_mutex 拷贝，可随时调用）。
         *        空闲设备在前、被客户端占用的在后
         */
        std::vector<DeviceSnapshot> list_device_snapshots();

    protected:
        Server server;

        void on_session_exit();
        void remove_gone_device(usb_device_handle_t dev);

        static void client_event_callback(const usb_host_client_event_msg_t* event_msg, void* arg);

        std::atomic<bool> should_exit_client_event_thread = false;

        //不可在这个线程发送网络包
        std::thread client_event_thread;

        std::map<std::uint8_t, usb_device_handle_t> host_devices;
        std::shared_mutex all_host_devices_mutex;
        usb_host_client_handle_t host_client_handle;

        std::mutex thread_cfg_mutex;

        static const char* TAG;
    };
}
