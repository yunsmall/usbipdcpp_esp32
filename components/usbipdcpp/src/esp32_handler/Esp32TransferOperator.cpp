#include "esp32_handler/Esp32TransferOperator.h"

#include <algorithm>
#include <esp_log.h>

#include "esp32_handler/Esp32DeviceHandler.h"
#include "usbipdcpp/constant.h"
#include "usbipdcpp/utils/SmallVector.h"

namespace usbipdcpp
{

namespace
{

void log_heap_diag(const char* tag)
{
    SPDLOG_INFO("{} heap: free={}, min_free={}, dma_free={}, dma_max_block={}, psram_free={}, psram_max_block={}", tag,
                esp_get_free_heap_size(), esp_get_minimum_free_heap_size(),
                heap_caps_get_free_size(MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL),
                heap_caps_get_largest_free_block(MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL),
                heap_caps_get_free_size(MALLOC_CAP_SPIRAM),
                heap_caps_get_largest_free_block(MALLOC_CAP_SPIRAM));
}

constexpr const char* TAG = "Esp32TransferOperator";

} // anonymous namespace

Esp32TransferOperator::Esp32TransferOperator()
{
}

// ========== transfer_handle 操作实现 ==========

void* Esp32TransferOperator::alloc_transfer_handle(std::size_t buffer_length, int num_iso_packets,
                                                   const UsbIpHeaderBasic& header,
                                                   const SetupPacket& setup_packet)
{
    bool is_control = (header.ep == 0);
    bool is_in = (header.direction == static_cast<std::uint32_t>(UsbIpDirection::In));

    // 计算实际需要分配的大小
    std::size_t actual_buffer_length = buffer_length;

    if (is_control) {
        // 控制传输：需要在 buffer 开头留出 setup packet 空间
        actual_buffer_length = USB_SETUP_PACKET_SIZE + buffer_length;
    }
    else if (is_in && buffer_length > 0) {
        // Bulk/Interrupt IN 传输：需要对齐到 MPS
        auto it = endpoint_mps_map_.find(static_cast<std::uint8_t>(header.ep | 0x80));
        if (it != endpoint_mps_map_.end() && it->second > 0) {
            std::uint16_t mps = it->second;
            if (actual_buffer_length % mps != 0) {
                actual_buffer_length = ((actual_buffer_length + mps - 1) / mps) * mps;
            }
        }
    }

    SPDLOG_DEBUG("alloc_transfer_handle: buffer_length={}, num_iso_packets={}, ep={}, is_control={}, actual={}",
                 buffer_length, num_iso_packets, header.ep, is_control, actual_buffer_length);

    usb_transfer_t* transfer = nullptr;
    esp_err_t err = usb_host_transfer_alloc(actual_buffer_length, num_iso_packets, &transfer);
    if (err != ESP_OK) [[unlikely]] {
        SPDLOG_ERROR("usb_host_transfer_alloc failed: {}, actual={}", esp_err_to_name(err), actual_buffer_length);
        log_heap_diag(TAG);
        return nullptr;
    }
    return transfer;
}

std::size_t Esp32TransferOperator::get_actual_length(void* transfer_handle)
{
    auto* trx = static_cast<usb_transfer_t*>(transfer_handle);
    return trx->actual_num_bytes;
}

usbipdcpp::UsbIpIsoPacketDescriptor
Esp32TransferOperator::get_iso_descriptor(void* transfer_handle, int index)
{
    auto* trx = static_cast<usb_transfer_t*>(transfer_handle);
    auto& iso = trx->isoc_packet_desc[index];
    // ISO 包在 buffer 中连续存放，offset = 前面所有包的 num_bytes 累加
    unsigned offset = 0;
    for (int i = 0; i < index; i++) {
        offset += trx->isoc_packet_desc[i].num_bytes;
    }
    return UsbIpIsoPacketDescriptor{
            .offset = offset,
            .length = static_cast<std::uint32_t>(iso.num_bytes),
            .actual_length = static_cast<std::uint32_t>(iso.actual_num_bytes),
            .status = static_cast<std::uint32_t>(Esp32DeviceHandler::trxstat2error(iso.status)),
    };
}

void Esp32TransferOperator::set_iso_descriptor(void* transfer_handle, int index,
                                               const UsbIpIsoPacketDescriptor& desc)
{
    auto* trx = static_cast<usb_transfer_t*>(transfer_handle);
    auto& iso = trx->isoc_packet_desc[index];
    iso.status = Esp32DeviceHandler::error2trxstat(desc.status);
    iso.actual_num_bytes = desc.actual_length;
    iso.num_bytes = desc.length;
}

void Esp32TransferOperator::free_transfer_handle(void* transfer_handle)
{
    usb_host_transfer_free(static_cast<usb_transfer_t*>(transfer_handle));
}

void Esp32TransferOperator::send_transfer_data(void* handle, asio::ip::tcp::socket& sock,
                                               std::size_t length,
                                               std::error_code& ec)
{
    auto* trx = static_cast<usb_transfer_t*>(handle);
    if (trx->num_isoc_packets > 0) {
        // ISO：发送描述符数组 + IN 数据（参考 LibusbTransferOperator 同逻辑）。
        // offset: buffer 中的包槽位偏移（pkt.num_bytes 步长，槽位大小由客户端
        // CMD_SUBMIT 的描述符 length 决定），同时用于数据读取和描述符 offset 字段。
        // 只对 IN 方向发送数据：与内核 stub_tx.c 一致（ISO 的 transfer buffer
        // 分支全部要求 usb_pipein），vhci 侧对 OUT 传输也不读数据；OUT 方向只发
        // 描述符。vhci 按 header 的 number_of_packets 读取描述符，不发会错位
        bool is_in = (trx->bEndpointAddress & 0x80) != 0;
        bool need_to_send_buffer = is_in && (length > 0);
        std::uint32_t offset = 0;
        SmallVector<asio::const_buffer, 130> buffers;
        SmallVector<decltype(UsbIpIsoPacketDescriptor{}.to_bytes()), 130> desc_bytes;
        for (int i = 0; i < trx->num_isoc_packets; i++) {
            auto &pkt = trx->isoc_packet_desc[i];
            if (need_to_send_buffer)
                buffers.push_back(asio::buffer(trx->data_buffer + offset, pkt.actual_num_bytes));
            UsbIpIsoPacketDescriptor desc{
                    .offset = offset,
                    .length = static_cast<std::uint32_t>(pkt.num_bytes),
                    .actual_length = static_cast<std::uint32_t>(pkt.actual_num_bytes),
                    .status = static_cast<std::uint32_t>(Esp32DeviceHandler::trxstat2error(pkt.status)),
            };
            desc_bytes.push_back(desc.to_bytes());
            offset += pkt.num_bytes;
        }
        for (auto &bytes: desc_bytes) {
            buffers.push_back(asio::buffer(bytes));
        }
        asio::write(sock, buffers, ec);
    }
    else {
        // 控制传输使用端点 0（地址 0x00 或 0x80），需要跳过 setup 包
        auto offset = ((trx->bEndpointAddress & 0x7F) == 0) ? USB_SETUP_PACKET_SIZE : 0;
        asio::write(sock, asio::buffer(reinterpret_cast<const char*>(trx->data_buffer) + offset, length), ec);
    }
}

void Esp32TransferOperator::recv_transfer_data(void* handle, asio::ip::tcp::socket& sock,
                                               std::size_t length,
                                               std::error_code& ec)
{
    auto* trx = static_cast<usb_transfer_t*>(handle);
    // 控制传输使用端点 0（地址 0x00 或 0x80），需要跳过 setup 包
    auto offset = ((trx->bEndpointAddress & 0x7F) == 0) ? USB_SETUP_PACKET_SIZE : 0;
    asio::read(sock, asio::buffer(static_cast<std::uint8_t*>(trx->data_buffer) + offset, length), ec);
    if (ec)
        return;

    // 校验并读取 ISO 描述符：length/actual_length 是客户端可控字段，必须验证
    // 才能写入 usb_transfer——usb_host 按 num_bytes 从缓冲区读写数据，length
    // 总和超过 data_buffer_size 会越界读写，actual_length 超过 length 则包数据
    // 溢出。不合法拒绝整个命令（调用方 ec 非空时抛异常断开连接）
    std::uint64_t total_length = 0;
    for (int i = 0; i < trx->num_isoc_packets; i++) {
        UsbIpIsoPacketDescriptor iso_desc{};
        iso_desc.from_socket(sock);
        // 校验写成 total_length + length > data_buffer_size：若写成
        // length > data_buffer_size - total_length，total_length 超过
        // data_buffer_size 时无符号减法会下溢成巨大值、校验失效（前序校验
        // 保证 total_length 恒 ≤ data_buffer_size，实际不会触发，但可读性差）
        if (iso_desc.actual_length > iso_desc.length ||
            total_length + iso_desc.length > static_cast<std::uint64_t>(trx->data_buffer_size)) [[unlikely]] {
            SPDLOG_ERROR("ISO 描述符非法：包 {} length={} actual_length={}（剩余缓冲 {}）",
                         i, iso_desc.length, iso_desc.actual_length,
                         static_cast<std::uint64_t>(trx->data_buffer_size) - total_length);
            ec = std::make_error_code(std::errc::invalid_argument);
            return;
        }
        total_length += iso_desc.length;
        // 客户端描述符里的 offset 被丢弃（set_iso_descriptor 不写它）：ESP32
        // 的 isoc_packet_desc 没有 offset 字段（buffer 布局隐式连续），发送端
        // 的 offset 由 send_transfer_data 按 num_bytes 累加自行计算，恶意或
        // 错误的 offset 无法影响服务器的数据定位。协议线格式数据本来就是
        // 紧凑的（内核 usbip_common.c 的 usbip_alloc_iso_desc_pdu 按 length
        // 连续累加），不存在带间隙的线上布局
        set_iso_descriptor(handle, i, iso_desc);
    }
}

} // namespace usbipdcpp