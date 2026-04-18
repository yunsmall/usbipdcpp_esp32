#pragma once

#include <atomic>
#include <condition_variable>
#include <mutex>

#include <asio.hpp>
#include <usb/usb_host.h>

#include "DeviceHandler/DeviceHandler.h"
#include "SetupPacket.h"
#include "utils/ConcurrentTransferTracker.h"
#include "utils/ObjectPool.h"
#include "esp32_handler/tools.h"

namespace usbipdcpp
{
    class Esp32DeviceHandler : public AbstDeviceHandler
    {
        friend class Esp32Server;

    public:
        Esp32DeviceHandler(UsbDevice& handle_device, usb_device_handle_t native_handle,
                           usb_host_client_handle_t host_client_handle);

        ~Esp32DeviceHandler() override;

        void on_new_connection(Session& current_session, error_code& ec) override;
        void on_disconnection(error_code& ec) override;
        void handle_unlink_seqnum(std::uint32_t unlink_seqnum, std::uint32_t cmd_seqnum) override;

#ifdef USBIPDCPP_ENABLE_BUSY_WAIT
        bool has_pending_transfers() const override
        {
            return transfer_tracker_.concurrent_count() > 0;
        }
#endif

        bool is_device_removed() const override
        {
            return device_removed_;
        }

        void on_device_removed() override
        {
            device_removed_ = true;
        }

    protected:
        void handle_control_urb(std::uint32_t seqnum, const UsbEndpoint& ep,
                                std::uint32_t transfer_flags, std::uint32_t transfer_buffer_length,
                                const SetupPacket& setup_packet, data_type&& transfer_buffer,
                                std::error_code& ec) override;
        void handle_bulk_transfer(std::uint32_t seqnum, const UsbEndpoint& ep,
                                  UsbInterface& interface, std::uint32_t transfer_flags,
                                  std::uint32_t transfer_buffer_length, data_type&& transfer_buffer,
                                  std::error_code& ec) override;
        void handle_interrupt_transfer(std::uint32_t seqnum, const UsbEndpoint& ep,
                                       UsbInterface& interface, std::uint32_t transfer_flags,
                                       std::uint32_t transfer_buffer_length, data_type&& transfer_buffer,
                                       std::error_code& ec) override;

        void handle_isochronous_transfer(std::uint32_t seqnum,
                                         const UsbEndpoint& ep, UsbInterface& interface,
                                         std::uint32_t transfer_flags,
                                         std::uint32_t transfer_buffer_length,
                                         data_type&& transfer_buffer,
                                         const std::vector<UsbIpIsoPacketDescriptor>& iso_packet_descriptors,
                                         std::error_code& ec) override;
        void cancel_all_transfer();
        void cancel_endpoint_all_transfers(uint8_t bEndpointAddress);

        //防止还没结束恢复端点状态就重新提交导致状态错误
        std::shared_mutex endpoint_cancellation_mutex;

        /**
         * @brief 发生错误代表没成功传输，设备未收到消息
         * @param setup_packet
         * @return
         */
        esp_err_t sync_control_transfer(const SetupPacket& setup_packet) const;

        int tweak_clear_halt_cmd(const SetupPacket& setup_packet);
        int tweak_set_interface_cmd(const SetupPacket& setup_packet);
        int tweak_set_configuration_cmd(const SetupPacket& setup_packet);
        int tweak_reset_device_cmd(const SetupPacket& setup_packet);

        /**
         * @brief 处理特殊控制请求
         * @param setup_packet
         * @return -1: 不需要 tweak，应该提交 transfer
         *          0: tweak 成功，不需要提交 transfer
         *         >0: tweak 失败（esp 错误码），不需要提交 transfer
         */
        int tweak_special_requests(const SetupPacket& setup_packet);

        static uint8_t get_esp32_transfer_flags(uint32_t in);

        static int trxstat2error(usb_transfer_status_t trxstat);
        static usb_transfer_status_t error2trxstat(int e);

        struct esp32_callback_args
        {
            Esp32DeviceHandler* handler = nullptr;
            std::uint32_t seqnum;
            usb_transfer_type_t transfer_type;
            bool is_out;
            std::uint32_t original_transfer_buffer_length = 0;  // 保存原始请求长度
        };

        static void transfer_callback(usb_transfer_t* trx);

        // 清理传输资源（供发送线程调用）
        static void cleanup_transfer_resources(void* context, void* transfer);

        // 对象池：64个
        using CallbackArgsPool = ObjectPool<esp32_callback_args, 64, true>;
        CallbackArgsPool callback_args_pool_;

        // 用于等待所有传输完成
        std::mutex transfer_complete_mutex_;
        std::condition_variable transfer_complete_cv_;

        // 分段锁传输追踪器
        ConcurrentTransferTracker<usb_transfer_t*> transfer_tracker_;

        static const char* TAG;

        usb_device_handle_t native_handle;
        usb_device_info_t device_info{};
        usb_host_client_handle_t host_client_handle;

        std::atomic_bool all_transfer_should_stop = false;

        std::atomic_bool device_removed_ = false;
    };
}
