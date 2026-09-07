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
    else if (is_in && buffer_length > 0 && num_iso_packets == 0) {
        // Bulk/Interrupt IN 传输：需要对齐到 MPS。ISO 跳过：usbh 的
        // transfer_check_usb_compliance 要求 ISO 的 num_bytes 精确等于各包
        // num_bytes 之和，对齐会致提交失败（依据 ESP-IDF v5.5 usbh.c）
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
    // 按 CMD_SUBMIT header 落库端点地址（ep 号 | 方向位，header.ep 已由协议层
    // 校验 ≤ 0x7F）：协议解析阶段（from_socket → recv_transfer_data）早于
    // receive_urb 设置 bEndpointAddress，而 usb_host_transfer_alloc 清零分配
    // （urb_alloc 用 calloc），此刻不写地址则 recv 无法区分控制/非控制传输。
    // receive_urb 提交前会按解析出的真实端点覆盖此值（两者正常情况下一致，
    // 客户端方向与端点矛盾时以真实端点为准，方向判断只影响线上数据收发）
    transfer->bEndpointAddress = static_cast<std::uint8_t>(header.ep) | (is_in ? 0x80 : 0);
    return transfer;
}

std::size_t Esp32TransferOperator::get_actual_length(void* transfer_handle)
{
    auto* trx = static_cast<usb_transfer_t*>(transfer_handle);
    return trx->actual_num_bytes;
}

bool Esp32TransferOperator::transfer_is_in(void* transfer_handle)
{
    auto* trx = static_cast<usb_transfer_t*>(transfer_handle);
    // 方向以端点地址的方向位为准。地址在 alloc_transfer_handle 时按 CMD_SUBMIT
    // 的 direction 落库，传输回调前 receive_urb 又按解析出的真实端点覆盖，
    // 因此查询时刻（RET_SUBMIT::to_socket，回调已执行）读到的必是真实方向。
    // 协议层按它决定回发长度：IN 回发 actual_length，OUT 恒 0（OUT 数据不回发）
    return (trx->bEndpointAddress & 0x80) != 0;
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
        // 描述符。vhci 按 header 的 number_of_packets 读取描述符，不发会错位。
        // length 已由协议层按方向算好（transfer_is_in 查询，OUT 恒传 0，
        // 见 RET_SUBMIT::to_socket），这里只按 length > 0 决定是否发数据，
        // 不再自行判断方向
        bool need_to_send_buffer = (length > 0);
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
    else if (length > 0) {
        // 控制传输使用端点 0（地址 0x00 或 0x80），需要跳过 setup 包。
        // 本函数在传输回调（receive_urb 已设置 bEndpointAddress）之后调用，
        // 端点地址此时必已填好，可直接按它判断
        auto offset = ((trx->bEndpointAddress & 0x7F) == 0) ? USB_SETUP_PACKET_SIZE : 0;
        asio::write(sock, asio::buffer(reinterpret_cast<const char*>(trx->data_buffer) + offset, length), ec);
    }
}

void Esp32TransferOperator::recv_transfer_data(void* handle, asio::ip::tcp::socket& sock,
                                               std::size_t length,
                                               std::error_code& ec)
{
    auto* trx = static_cast<usb_transfer_t*>(handle);
    // 控制传输 buffer 前 8 字节留给 setup 包（alloc 时多分配了
    // USB_SETUP_PACKET_SIZE），由后续 receive_urb 填入；此处从偏移 8 开始
    // 读取数据阶段内容。bEndpointAddress 已由 alloc_transfer_handle 按
    // CMD_SUBMIT header 的 ep+direction 落库（见该函数注释），可据此判断
    // 是否控制传输（recv 自身无法用其他字段推断：usbh 提交前不设置任何
    // 传输类型/端点信息，transfer 结构由 calloc 清零分配）
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