#pragma once

#include <map>
#include <shared_mutex>

#include <asio.hpp>
#include <usb/usb_host.h>

#include "DeviceHandler/DeviceHandler.h"
#include "SetupPacket.h"
#include "constant.h"
#include "esp32_handler/tools.h"

namespace usbipdcpp
{
    class Esp32DeviceHandler : public DeviceHandlerBase
    {
        friend class Esp32Server;

    public:
        Esp32DeviceHandler(UsbDevice& handle_device, usb_device_handle_t native_handle,
                           usb_host_client_handle_t host_client_handle);

        ~Esp32DeviceHandler() override;

        void on_new_connection(error_code& ec) override;
        void on_disconnection(error_code& ec) override;
        void handle_unlink_seqnum(std::uint32_t seqnum) override;

    protected:
        void handle_control_urb(Session& session,
                                std::uint32_t seqnum, const UsbEndpoint& ep,
                                std::uint32_t transfer_flags, std::uint32_t transfer_buffer_length,
                                const SetupPacket& setup_packet, const data_type& req, std::error_code& ec) override;
        void handle_bulk_transfer(Session& session, std::uint32_t seqnum, const UsbEndpoint& ep,
                                  UsbInterface& interface, std::uint32_t transfer_flags,
                                  std::uint32_t transfer_buffer_length, const data_type& out_data,
                                  std::error_code& ec) override;
        void handle_interrupt_transfer(Session& session, std::uint32_t seqnum, const UsbEndpoint& ep,
                                       UsbInterface& interface, std::uint32_t transfer_flags,
                                       std::uint32_t transfer_buffer_length, const data_type& out_data,
                                       std::error_code& ec) override;

        void handle_isochronous_transfer(Session& session, std::uint32_t seqnum,
                                         const UsbEndpoint& ep, UsbInterface& interface,
                                         std::uint32_t transfer_flags,
                                         std::uint32_t transfer_buffer_length,
                                         const data_type& req,
                                         const std::vector<UsbIpIsoPacketDescriptor>& iso_packet_descriptors,
                                         std::error_code& ec) override;
        /**
         * @brief 发生错误代表没成功传输，设备未收到消息
         * @param setup_packet
         * @return
         */
        esp_err_t sync_control_transfer(const SetupPacket& setup_packet) const;

        esp_err_t tweak_clear_halt_cmd(const SetupPacket& setup_packet);
        esp_err_t tweak_set_interface_cmd(const SetupPacket& setup_packet);
        esp_err_t tweak_set_configuration_cmd(const SetupPacket& setup_packet);
        esp_err_t tweak_reset_device_cmd(const SetupPacket& setup_packet);

        /**
         * @brief 返回是否做了特殊操作
         * @param setup_packet
         * @return
         */
        bool tweak_special_requests(const SetupPacket& setup_packet);

        static uint8_t get_esp32_transfer_flags(uint32_t in);

        static int trxstat2error(usb_transfer_status_t trxstat);
        static usb_transfer_status_t error2trxstat(int e);

        struct esp32_callback_args
        {
            Esp32DeviceHandler& handler;
            Session& session;
            std::uint32_t seqnum;
            usb_transfer_type_t transfer_type;
            bool is_out;
        };

        static void transfer_callback(usb_transfer_t* trx);

        static const char* TAG;

        std::map<std::uint32_t, usb_transfer_t*> transferring_data;
        std::shared_mutex transferring_data_mutex;

        usb_device_handle_t native_handle;
        usb_device_info_t device_info{};
        usb_host_client_handle_t host_client_handle;

        std::atomic_bool all_transfer_should_stop = false;

        std::atomic_bool has_device = true;
    };
}
