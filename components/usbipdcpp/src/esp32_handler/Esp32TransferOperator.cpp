#include "esp32_handler/Esp32TransferOperator.h"

#include <algorithm>
#include <esp_log.h>

#include "esp32_handler/Esp32DeviceHandler.h"
#include "usbipdcpp/constant.h"

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
#ifdef CONFIG_USBIPD_ENABLE_CHUNKED_TRANSFER
    enable_chunking = CONFIG_USBIPD_ENABLE_CHUNKED_TRANSFER;
#else
    enable_chunking = false;
#endif

    // 完全不知道为什么分块会导致某些设备比如U盘、jlink等传输出现问题，故这里关闭了
    // 如果有知道怎么修的请帮忙提个PR
    enable_chunking = false;

#ifdef CONFIG_USBIPD_MAX_CHUNK_SIZE
    chunk_size_ = CONFIG_USBIPD_MAX_CHUNK_SIZE;
#endif
    // chunk_size_ = 4096;
    chunk_size_ = 16384;
}

// ========== transfer_handle 操作实现 ==========

void* Esp32TransferOperator::alloc_transfer_handle(std::size_t buffer_length, int num_iso_packets,
                                                   const UsbIpHeaderBasic& header,
                                                   const SetupPacket& setup_packet)
{
    bool is_control = (header.ep == 0);
    bool is_in = (header.direction == static_cast<std::uint32_t>(UsbIpDirection::In));

    // 分块：启用、非控制、非等时，且超过配置的阈值
    if (enable_chunking && !is_control && num_iso_packets == 0 && buffer_length > static_cast<std::size_t>(
            chunk_size_)) {
        std::size_t chunk_size = chunk_size_;
        std::uint16_t mps = 0;
        if (is_in) {
            auto it = endpoint_mps_map_.find(static_cast<std::uint8_t>(header.ep | 0x80));
            if (it != endpoint_mps_map_.end())
                mps = it->second;
        }

        auto* ct = chunked_pool_.alloc();
        if (!ct)
            ct = new ChunkedTransfer{};
        ct->transfers.clear();
        std::size_t remaining = buffer_length;
        std::size_t total_aligned = 0;
        while (remaining > 0) {
            std::size_t alloc_size = std::min(remaining, chunk_size);
            if (mps > 0 && alloc_size % mps != 0)
                alloc_size = ((alloc_size + mps - 1) / mps) * mps;
            total_aligned += alloc_size;
            usb_transfer_t* trx = nullptr;
            esp_err_t err = usb_host_transfer_alloc(alloc_size, 0, &trx);
            if (err != ESP_OK) [[unlikely]] {
                SPDLOG_ERROR("Chunk alloc failed: {}, size={}, chunk={}/{}",
                             esp_err_to_name(err), alloc_size,
                             ct->transfers.size() + 1, (buffer_length + chunk_size - 1) / chunk_size);
                log_heap_diag(TAG);
                for (auto* t: ct->transfers)
                    usb_host_transfer_free(t);
                ct->transfers.clear();
                if (!chunked_pool_.free(ct))
                    delete ct;
                return nullptr;
            }
            ct->transfers.push_back(trx);
            remaining -= std::min(remaining, chunk_size);
        }
        if (is_in) {
            ct->in_data = static_cast<uint8_t*>(heap_caps_malloc(total_aligned, MALLOC_CAP_SPIRAM));
            if (!ct->in_data) [[unlikely]] {
                SPDLOG_ERROR("Chunk IN buffer alloc failed: spiram, size={}", total_aligned);
                log_heap_diag(TAG);
                for (auto* t: ct->transfers)
                    usb_host_transfer_free(t);
                ct->transfers.clear();
                if (!chunked_pool_.free(ct))
                    delete ct;
                return nullptr;
            }
            ct->in_data_size = total_aligned;
        }
        SPDLOG_INFO("CHUNKED seqnum={} ep={:02x} {} total={} chunks={} chunk_size={} in_buf={}",
                    header.seqnum, header.ep, is_in ? "IN" : "OUT",
                    buffer_length, ct->transfers.size(), chunk_size,
                    static_cast<void*>(ct->in_data));
        std::lock_guard lock(chunked_transfers_mutex_);
        // 拒绝重复 seqnum：分块表以 seqnum 为 key，重复会覆盖旧条目，
        // 旧 ChunkedTransfer 泄漏且其回调/取消语义错乱。协议要求 seqnum
        // 单调递增（内核 vhci 原子递增），重复属客户端违规，直接中止本次
        // 传输（返回 nullptr 由协议层报错断开连接，参考 protocol.cpp 的
        // alloc 失败处理）
        if (chunked_transfers_.contains(header.seqnum)) [[unlikely]] {
            SPDLOG_ERROR("CHUNKED 重复 seqnum={}，中止本次传输", header.seqnum);
            for (auto* t: ct->transfers)
                usb_host_transfer_free(t);
            ct->transfers.clear();
            if (ct->in_data)
                heap_caps_free(ct->in_data);
            ct->in_data = nullptr;
            if (!chunked_pool_.free(ct))
                delete ct;
            return nullptr;
        }
        chunked_transfers_.emplace(header.seqnum, ct);
        return ct;
    }

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
    if (!enable_chunking) {
        return static_cast<usb_transfer_t*>(transfer_handle)->actual_num_bytes;
    }
    std::lock_guard lock(chunked_transfers_mutex_);
    if (auto* ct = try_get_chunked(transfer_handle)) {
        return ct->total_actual_length.load();
    }
    auto* trx = static_cast<usb_transfer_t*>(transfer_handle);
    return trx->actual_num_bytes;
}

usbipdcpp::UsbIpIsoPacketDescriptor
Esp32TransferOperator::get_iso_descriptor(void* transfer_handle, int index)
{
    auto* trx = static_cast<usb_transfer_t*>(transfer_handle);
    auto& iso = trx->isoc_packet_desc[index];
    return UsbIpIsoPacketDescriptor{
            .offset = 0, // 需要调用方计算
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
    if (!enable_chunking) {
        usb_host_transfer_free(static_cast<usb_transfer_t*>(transfer_handle));
        return;
    }
    std::lock_guard lock(chunked_transfers_mutex_);
    if (auto* ct = try_get_chunked(transfer_handle)) {
        for (auto* t: ct->transfers)
            if (t)
                usb_host_transfer_free(t);
        ct->transfers.clear();
        // heap_caps_free(nullptr) 本身安全，这里显式判空让意图更明确
        if (ct->in_data)
            heap_caps_free(ct->in_data);
        ct->in_data = nullptr;
        ct->in_short = false;
        ct->transfer_started = false;
        // 从 map 中移除（按 value 找到 key）
        for (auto it = chunked_transfers_.begin(); it != chunked_transfers_.end(); ++it) {
            if (it->second == ct) {
                chunked_transfers_.erase(it);
                break;
            }
        }
        if (!chunked_pool_.free(ct))
            delete ct;
        return;
    }
    usb_host_transfer_free(static_cast<usb_transfer_t*>(transfer_handle));
}

void Esp32TransferOperator::send_transfer_data(void* handle, asio::ip::tcp::socket& sock,
                                               std::size_t length,
                                               std::error_code& ec)
{
    {
        std::lock_guard lock(chunked_transfers_mutex_);
        if (auto* ct = try_get_chunked(handle)) {
            if (ct->in_data) {
                asio::write(sock, asio::buffer(
                                    reinterpret_cast<const char*>(ct->in_data), length), ec);
            }
            else {
                std::size_t remaining = length;
                for (std::size_t i = 0; i < ct->transfers.size() && remaining > 0 && !ec; i++) {
                    auto* trx = ct->transfers[i];
                    // 已完成的分块在回调中被置 null 并释放，防御性跳过
                    // （正常 IN 分块必有 in_data 不会走到此分支，仅防御
                    // in_data 意外为空的路径）
                    if (!trx) [[unlikely]]
                        continue;
                    std::size_t chunk_len = std::min(
                            remaining, static_cast<std::size_t>(trx->actual_num_bytes));
                    if (chunk_len > 0) {
                        asio::write(sock, asio::buffer(
                                            reinterpret_cast<const char*>(trx->data_buffer), chunk_len),
                                    ec);
                        remaining -= chunk_len;
                    }
                }
            }
            return;
        }
    }
    auto* trx = static_cast<usb_transfer_t*>(handle);
    // 控制传输使用端点 0（地址 0x00 或 0x80），需要跳过 setup 包
    auto offset = ((trx->bEndpointAddress & 0x7F) == 0) ? USB_SETUP_PACKET_SIZE : 0;
    asio::write(sock, asio::buffer(reinterpret_cast<const char*>(trx->data_buffer) + offset, length), ec);
}

void Esp32TransferOperator::recv_transfer_data(void* handle, asio::ip::tcp::socket& sock,
                                               std::size_t length,
                                               std::error_code& ec)
{
    {
        std::lock_guard lock(chunked_transfers_mutex_);
        if (auto* ct = try_get_chunked(handle)) {
            std::size_t remaining = length;
            for (auto* trx: ct->transfers) {
                if (remaining == 0)
                    break;
                std::size_t chunk_len = std::min(remaining, trx->data_buffer_size);
                asio::read(sock, asio::buffer(trx->data_buffer, chunk_len), ec);
                if (ec)
                    return;
                trx->num_bytes = chunk_len;
                remaining -= chunk_len;
            }
            return;
        }
    }
    auto* trx = static_cast<usb_transfer_t*>(handle);
    auto offset = ((trx->bEndpointAddress & 0x7F) == 0) ? USB_SETUP_PACKET_SIZE : 0;
    asio::read(sock, asio::buffer(static_cast<std::uint8_t*>(trx->data_buffer) + offset, length), ec);
}

ChunkedTransfer* Esp32TransferOperator::try_get_chunked(void* handle)
{
    for (auto& p: chunked_transfers_) {
        if (p.second == handle)
            return p.second;
    }
    return nullptr;
}

} // namespace usbipdcpp