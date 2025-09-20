#pragma once

#include <usb/usb_host.h>

#include <shared_mutex>

#include <asio.hpp>


#include "Server.h"

namespace usbipdcpp
{
    class Esp32Server : public Server
    {
    public:
        Esp32Server();

        void init_client();
        void bind_host_device(usb_device_handle_t dev);
        void unbind_host_device(usb_device_handle_t device);
        void start(asio::ip::tcp::endpoint& ep) override;
        void stop() override;

        ~Esp32Server() override;

    protected:
        void on_session_exit() override;
        void if_is_esp32_then_mark_removed(std::shared_ptr<AbstDeviceHandler> handler);
        void remove_gone_device(usb_device_handle_t dev);

        static void client_event_callback(const usb_host_client_event_msg_t* event_msg, void* arg);

        std::atomic<bool> should_exit_client_event_thread = false;

        //不可在这个线程发送网络包
        std::thread client_event_thread;

        std::map<std::uint8_t, usb_device_handle_t> host_devices;
        std::shared_mutex all_host_devices_mutex;
        usb_host_client_handle_t host_client_handle;

        static const char* TAG;
    };
}
