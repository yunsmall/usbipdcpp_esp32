#pragma once

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <mutex>
#include <unordered_map>
#include <vector>

#include <usb/usb_host.h>

#include "usbipdcpp/DeviceHandler/TransferOperator.h"
#include "usbipdcpp/protocol.h"
#include "usbipdcpp/utils/ObjectPool.h"

namespace usbipdcpp
{

// 回调参数结构定义在 Esp32DeviceHandler.h，此处前向声明供 ChunkedTransfer 使用
struct esp32_callback_args;

/**
 * @brief ESP32 usb_host 后端的 TransferOperator 实现
 *
 * 新版 usbipdcpp 将 transfer_handle 操作从 AbstDeviceHandler 解耦为
 * TransferOperator 抽象，由 get_cmd_from_socket 在解析 CMD_SUBMIT 时通过
 * transfer.set_operator() 注入，协议反序列化与 RET_SUBMIT 发送均走本类。
 * 本类只包含原先 Esp32DeviceHandler 中 transfer_handle 相关的内容
 * （分配/释放/读写 + 分块传输状态），状态转换工具（trxstat2error 等）
 * 保留在 Esp32DeviceHandler，直接以静态方式调用（参考 LibusbTransferOperator）。
 */
class Esp32TransferOperator : public TransferOperator
{
public:
    Esp32TransferOperator();
    ~Esp32TransferOperator() override = default;

    void* alloc_transfer_handle(std::size_t buffer_length, int num_iso_packets,
                                const UsbIpHeaderBasic& header, const SetupPacket& setup_packet) override;
    void free_transfer_handle(void* handle) override;
    std::size_t get_actual_length(void* handle) override;
    UsbIpIsoPacketDescriptor get_iso_descriptor(void* handle, int index) override;
    void set_iso_descriptor(void* handle, int index, const UsbIpIsoPacketDescriptor& desc) override;
    void send_transfer_data(void* handle, asio::ip::tcp::socket& sock,
                            std::size_t length, std::error_code& ec) override;
    void recv_transfer_data(void* handle, asio::ip::tcp::socket& sock,
                            std::size_t length, std::error_code& ec) override;

    // 端点 MPS 查找表：端点地址 -> max_packet_size
    std::unordered_map<std::uint8_t, std::uint16_t> endpoint_mps_map_;
};

} // namespace usbipdcpp