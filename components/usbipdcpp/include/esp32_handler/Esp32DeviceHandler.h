#pragma once

#include <atomic>
#include <condition_variable>
#include <mutex>
#include <queue>
#include <set>
#include <shared_mutex>
#include <unordered_map>
#include <vector>

#include <asio.hpp>
#include <usb/usb_host.h>

#include "DeviceHandler/DeviceHandler.h"
#include "SetupPacket.h"
#include "utils/ObjectPool.h"
#include "esp32_handler/tools.h"

namespace usbipdcpp
{

// 分块传输结构：当 transfer_buffer_length > CONFIG_USBIPD_MAX_CHUNK_SIZE 时，
// 将一个大传输拆成多个 usb_transfer_t，全部完成后统一发 ret_submit。
// 所有 chunk 共用一个 esp32_callback_args（trx->context 指向同一个 cb），
// cb->transfer 持有 TransferHandle，cb->chunked 指向本结构。
struct ChunkedTransfer {
    // 有序存放的每个分块 transfer，按序提交、按序发送
    std::vector<usb_transfer_t*> transfers;
    // USB/IP 命令序列号，用于 tracker 和 chunked_transfers_ 表查找
    std::uint32_t seqnum = 0;
    // true = OUT（主机→设备），false = IN（设备→主机）
    bool is_out = false;
    // 原始 CMD_SUBMIT 请求的 transfer_buffer_length，仅用于日志和 OUT 出错时参考
    std::size_t transfer_buffer_length = 0;
    // 所有已完成 chunk 中的最差 USB 传输状态，COMPLETED=0 最优，数字越大越差
    usb_transfer_status_t worst_status = USB_TRANSFER_STATUS_COMPLETED;
    // 未完成的 chunk 数量，原子操作，每个 chunk 回调减 1，归零时触发 ret_submit/ret_unlink
    std::atomic<int> pending_count{0};
    // IN：设备实际返回的字节总和；OUT：设备实际确认收到的字节总和
    std::atomic<std::size_t> total_actual_length{0};
    // 首个分块成功传输后置 true，handle_unlink 据此决定不 cancel，回调据此决定重提交/最终响应
    std::atomic<bool> transfer_started{false};
    // 端点地址，避免 transfers[0] 被提前释放后无法访问
    std::uint8_t ep_address = 0;
    // IN：PSRAM 累积缓冲区，每个分块完成后将设备返回数据拷入对应位置，
    // 使 DMA transfer 可以立即释放，ret_submit 时从此缓冲区发送
    uint8_t* in_data = nullptr;
    // IN 累积缓冲区的总大小（= 原始 transfer_buffer_length）
    std::size_t in_data_size = 0;
    // IN：设备在某分块返回短包/ZLP 后置 true。
    // CANCELED 回调据此跳过重提交（设备已结束，重提交只会继续 NAK）
    std::atomic<bool> in_short{false};
    // 下一个待提交的分块索引，每次仅提交一个，完成后回调里递进
    int current_chunk = 0;
    // 原始 CMD_SUBMIT 的 transfer_flags，回调提交后续分块时计算 ZLP 位
    std::uint32_t transfer_flags = 0;
};

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

        bool is_device_removed() const override
        {
            return device_removed_;
        }

        void on_device_removed() override
        {
            device_removed_ = true;
        }

        void receive_urb(UsbIpCommand::UsbIpCmdSubmit cmd,
                         UsbEndpoint ep,
                         std::optional<UsbInterface> interface,
                         usbipdcpp::error_code &ec) override;

        // ========== transfer_handle 操作覆盖实现 ==========
        void* alloc_transfer_handle(std::size_t buffer_length, int num_iso_packets, const UsbIpHeaderBasic& header, const SetupPacket& setup_packet) override;
        void* get_transfer_buffer(void* transfer_handle) override;
        std::size_t get_actual_length(void* transfer_handle) override;
        std::size_t get_read_data_offset(void* transfer_handle) override;
        std::size_t get_write_data_offset(const UsbIpHeaderBasic& header) override;
        UsbIpIsoPacketDescriptor get_iso_descriptor(void* transfer_handle, int index) override;
        void set_iso_descriptor(void* transfer_handle, int index, const UsbIpIsoPacketDescriptor& desc) override;
        void free_transfer_handle(void* transfer_handle) override;

        void send_transfer_data(void* handle, asio::ip::tcp::socket& sock,
                                std::size_t length, std::error_code& ec) override;
        void recv_transfer_data(void* handle, asio::ip::tcp::socket& sock,
                                std::size_t length, std::error_code& ec) override;

    protected:
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
            ChunkedTransfer* chunked = nullptr;  // 分块传输的共享状态
            bool unlinked = false;                // CMD_UNLINK 已到达
            std::uint32_t unlink_cmd_seqnum = 0;
            std::uint32_t original_transfer_buffer_length = 0;
            TransferHandle transfer;  // 拥有 transfer 的所有权

            void reset()
            {
                handler = nullptr;
                seqnum = 0;
                transfer_type = static_cast<usb_transfer_type_t>(0);
                is_out = false;
                chunked = nullptr;
                unlinked = false;
                unlink_cmd_seqnum = 0;
                original_transfer_buffer_length = 0;
                transfer.reset();
            }
        };

        static void transfer_callback(usb_transfer_t* trx);
        static void chunked_transfer_callback(usb_transfer_t* trx);

        // 对象池：64个
        using CallbackArgsPool = ObjectPool<esp32_callback_args, 64, true>;
        CallbackArgsPool callback_args_pool_;

        // 用于等待所有传输完成
        std::mutex transfer_complete_mutex_;
        std::condition_variable transfer_complete_cv_;

        // 非分块传输表：seqnum → callback_args*（参考 libusb 模型）
        std::mutex transfers_mutex_;
        std::unordered_map<std::uint32_t, esp32_callback_args*> transfers_;
        std::atomic<std::size_t> pending_count_{0};

        // 分块传输表：seqnum → ChunkedTransfer*
        std::mutex chunked_transfers_mutex_;
        std::unordered_map<std::uint32_t, ChunkedTransfer*> chunked_transfers_;
        std::atomic<int> chunked_count_{0};

        // ChunkedTransfer 对象池，避免频繁 new/delete
        using ChunkedPool = ObjectPool<ChunkedTransfer, 16, true>;
        ChunkedPool chunked_pool_;

        // 同端点串行化：端点有分块传输正在处理时，新来的传输入队等待
        // 避免一次性提交多个分块阻塞 pipe，也防止多个传输交织打乱顺序
        struct DeferredUrb {
            UsbIpCommand::UsbIpCmdSubmit cmd;
            UsbEndpoint ep;
            std::optional<UsbInterface> interface;
        };
        std::mutex deferred_urbs_mutex_;
        std::unordered_map<uint8_t, std::queue<DeferredUrb>> deferred_urbs_;
        // 同端点分块传输进行中标记，防止多个分块传输同时占用同一 pipe
        std::mutex active_chunked_eps_mutex_;
        std::set<uint8_t> active_chunked_eps_;
        // 提交分块传输的第一个 chunk（从 receive_urb 或 deferred 出队后调用）
        void submit_first_chunk(ChunkedTransfer* ct, esp32_callback_args* cb,
                                const UsbEndpoint& ep, std::uint32_t transfer_flags);
        // 出队并处理指定端点的下一个 pending transfer
        void process_pending_urb(uint8_t ep_addr);

        static const char* TAG;

        usb_device_handle_t native_handle;
        usb_device_info_t device_info{};
        usb_host_client_handle_t host_client_handle;

        std::atomic_bool all_transfer_should_stop = false;

        std::atomic_bool device_removed_ = false;

        bool enable_chunking = true;
        std::size_t chunk_size_ = 4096;

        // 端点 MPS 查找表：端点地址 -> max_packet_size
        std::unordered_map<std::uint8_t, std::uint16_t> endpoint_mps_map_;
    };
}
