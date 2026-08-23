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

#include "usbipdcpp/DeviceHandler/DeviceHandler.h"
#include "usbipdcpp/SetupPacket.h"
#include "usbipdcpp/protocol.h"
#include "usbipdcpp/utils/ObjectPool.h"
#include "esp32_handler/Esp32TransferOperator.h"
#include "esp32_handler/tools.h"

namespace usbipdcpp
{

    class Esp32DeviceHandler;

    // 传输回调参数结构（原为 Esp32DeviceHandler 的嵌套类）。
    // 挪到命名空间级是为了让 Esp32TransferOperator.h 能前向声明它并放入
    // ChunkedTransfer::cb（嵌套类无法在类外前向声明）
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

        // ========== transfer 状态转换工具（Esp32TransferOperator 也会调用） ==========
        static uint8_t get_esp32_transfer_flags(uint32_t in);

        static int trxstat2error(usb_transfer_status_t trxstat);
        static usb_transfer_status_t error2trxstat(int e);

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

        // 分块传输计数（分块状态本身在 Esp32TransferOperator 中）
        std::atomic<int> chunked_count_{0};

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

        // transfer 层操作器（分块状态随 transfer_handle 逻辑迁入，所有权在基类）
        Esp32TransferOperator* transfer_operator_;
    };
}
