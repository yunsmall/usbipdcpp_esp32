#pragma once

#include <usb/usb_host.h>

#include <shared_mutex>
#include <mutex>

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
