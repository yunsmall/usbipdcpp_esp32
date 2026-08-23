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
    // 本笔分块传输对应的回调参数（receive_urb 设置）。
    // handle_unlink_seqnum 直接用此字段找 cb，而不是遍历 transfers 读
    // trx->context：回调完成路径会无锁地把已完成的 chunk 置 null 并释放
    // trx，遍历存在数据竞争，且全部置 null 后找不到 cb 会误发 ret_unlink(0)。
    esp32_callback_args* cb = nullptr;
};

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

    // ========== 分块传输状态（随 transfer 层迁入，Esp32DeviceHandler 协作访问） ==========
    bool enable_chunking = true;
    std::size_t chunk_size_ = 4096;

    // 分块传输表：seqnum → ChunkedTransfer*
    std::mutex chunked_transfers_mutex_;
    std::unordered_map<std::uint32_t, ChunkedTransfer*> chunked_transfers_;

    // ChunkedTransfer 对象池，避免频繁 new/delete
    using ChunkedPool = ObjectPool<ChunkedTransfer, 16, true>;
    ChunkedPool chunked_pool_;

    // 端点 MPS 查找表：端点地址 -> max_packet_size
    std::unordered_map<std::uint8_t, std::uint16_t> endpoint_mps_map_;

    // 按 value 查找分块传输（handle 指向 ChunkedTransfer* 时使用）
    ChunkedTransfer* try_get_chunked(void* handle);
};

} // namespace usbipdcpp