#include "esp32_handler/Esp32DeviceHandler.h"

#include <algorithm>
#include <esp_log.h>

#include "Session.h"
#include "protocol.h"
#include "SetupPacket.h"
#include "constant.h"
#include "Endpoint.h"
#include "utils/LatencyTracker.h"

const char *usbipdcpp::Esp32DeviceHandler::TAG = "Esp32DeviceHandler";

usbipdcpp::Esp32DeviceHandler::Esp32DeviceHandler(UsbDevice &handle_device, usb_device_handle_t native_handle,
                                                  usb_host_client_handle_t host_client_handle) :
    AbstDeviceHandler(handle_device), native_handle(native_handle), host_client_handle(host_client_handle) {
    custom_transfer_io = true;
#ifdef CONFIG_USBIPD_ENABLE_CHUNKED_TRANSFER
    enable_chunking = CONFIG_USBIPD_ENABLE_CHUNKED_TRANSFER;
#else
    enable_chunking = false;
#endif

    // enable_chunking = true;

#ifdef CONFIG_USBIPD_MAX_CHUNK_SIZE
    chunk_size_ = CONFIG_USBIPD_MAX_CHUNK_SIZE;
#endif
    // chunk_size_ = 4096;
    // chunk_size_ = 131072;
    ESP_ERROR_CHECK(usb_host_device_info(native_handle, &device_info));
}

usbipdcpp::Esp32DeviceHandler::~Esp32DeviceHandler() = default;

namespace {

usbipdcpp::ChunkedTransfer *try_get_chunked(std::unordered_map<std::uint32_t, usbipdcpp::ChunkedTransfer *> &map,
                                            void *handle) {
    for (auto &p: map) {
        if (p.second == handle)
            return p.second;
    }
    return nullptr;
}

void log_heap_diag(const char *tag) {
    SPDLOG_INFO("{} heap: free={}, min_free={}, dma_free={}, dma_max_block={}, psram_free={}, psram_max_block={}", tag,
                esp_get_free_heap_size(), esp_get_minimum_free_heap_size(),
                heap_caps_get_free_size(MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL),
                heap_caps_get_largest_free_block(MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL),
                heap_caps_get_free_size(MALLOC_CAP_SPIRAM),
                heap_caps_get_largest_free_block(MALLOC_CAP_SPIRAM));
}

} // anonymous namespace

// ========== transfer_handle 操作实现 ==========

void *usbipdcpp::Esp32DeviceHandler::alloc_transfer_handle(std::size_t buffer_length, int num_iso_packets,
                                                           const UsbIpHeaderBasic &header,
                                                           const SetupPacket &setup_packet) {
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

        auto *ct = chunked_pool_.alloc();
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
            usb_transfer_t *trx = nullptr;
            esp_err_t err = usb_host_transfer_alloc(alloc_size, 0, &trx);
            if (err != ESP_OK) [[unlikely]] {
                SPDLOG_ERROR("Chunk alloc failed: {}, size={}, chunk={}/{}",
                             esp_err_to_name(err), alloc_size,
                             ct->transfers.size() + 1, (buffer_length + chunk_size - 1) / chunk_size);
                log_heap_diag(TAG);
                for (auto *t: ct->transfers)
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
            ct->in_data = static_cast<uint8_t *>(heap_caps_malloc(total_aligned, MALLOC_CAP_SPIRAM));
            if (!ct->in_data) [[unlikely]] {
                SPDLOG_ERROR("Chunk IN buffer alloc failed: spiram, size={}", total_aligned);
                log_heap_diag(TAG);
                for (auto *t: ct->transfers)
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
        chunked_transfers_[header.seqnum] = ct;
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

    usb_transfer_t *transfer = nullptr;
    esp_err_t err = usb_host_transfer_alloc(actual_buffer_length, num_iso_packets, &transfer);
    if (err != ESP_OK) [[unlikely]] {
        SPDLOG_ERROR("usb_host_transfer_alloc failed: {}, actual={}", esp_err_to_name(err), actual_buffer_length);
        log_heap_diag(TAG);
        return nullptr;
    }
    return transfer;
}

void *usbipdcpp::Esp32DeviceHandler::get_transfer_buffer(void *transfer_handle) {
    std::lock_guard lock(chunked_transfers_mutex_);
    if (auto *ct = try_get_chunked(chunked_transfers_, transfer_handle)) {
        return ct->in_data ? ct->in_data : (ct->transfers.empty() ? nullptr : ct->transfers[0]->data_buffer);
    }
    auto *trx = static_cast<usb_transfer_t *>(transfer_handle);
    return trx->data_buffer;
}

std::size_t usbipdcpp::Esp32DeviceHandler::get_actual_length(void *transfer_handle) {
    std::lock_guard lock(chunked_transfers_mutex_);
    if (auto *ct = try_get_chunked(chunked_transfers_, transfer_handle)) {
        return ct->total_actual_length.load();
    }
    auto *trx = static_cast<usb_transfer_t *>(transfer_handle);
    return trx->actual_num_bytes;
}

std::size_t usbipdcpp::Esp32DeviceHandler::get_read_data_offset(void *transfer_handle) {
    std::lock_guard lock(chunked_transfers_mutex_);
    if (try_get_chunked(chunked_transfers_, transfer_handle)) {
        return 0; // 分块仅用于非控制传输，无需跳过 setup packet
    }
    auto *trx = static_cast<usb_transfer_t *>(transfer_handle);
    // 控制传输使用端点 0（地址 0x00 或 0x80）
    if ((trx->bEndpointAddress & 0x7F) == 0) {
        return USB_SETUP_PACKET_SIZE; // 8
    }
    return 0;
}

std::size_t usbipdcpp::Esp32DeviceHandler::get_write_data_offset(const UsbIpHeaderBasic &header) {
    // 控制传输 (ep == 0) 需要跳过 setup packet
    if (header.ep == 0) {
        return USB_SETUP_PACKET_SIZE;
    }
    return 0;
}

usbipdcpp::UsbIpIsoPacketDescriptor
usbipdcpp::Esp32DeviceHandler::get_iso_descriptor(void *transfer_handle, int index) {
    auto *trx = static_cast<usb_transfer_t *>(transfer_handle);
    auto &iso = trx->isoc_packet_desc[index];
    return UsbIpIsoPacketDescriptor{
            .offset = 0, // 需要调用方计算
            .length = static_cast<std::uint32_t>(iso.num_bytes),
            .actual_length = static_cast<std::uint32_t>(iso.actual_num_bytes),
            .status = static_cast<std::uint32_t>(trxstat2error(iso.status)),
            .length_in_transfer_buffer_only_for_send = static_cast<std::uint32_t>(iso.num_bytes)
    };
}

void usbipdcpp::Esp32DeviceHandler::set_iso_descriptor(void *transfer_handle, int index,
                                                       const UsbIpIsoPacketDescriptor &desc) {
    auto *trx = static_cast<usb_transfer_t *>(transfer_handle);
    auto &iso = trx->isoc_packet_desc[index];
    iso.status = error2trxstat(desc.status);
    iso.actual_num_bytes = desc.actual_length;
    iso.num_bytes = desc.length;
}

void usbipdcpp::Esp32DeviceHandler::free_transfer_handle(void *transfer_handle) {
    std::lock_guard lock(chunked_transfers_mutex_);
    if (auto *ct = try_get_chunked(chunked_transfers_, transfer_handle)) {
        for (auto *t: ct->transfers)
            if (t)
                usb_host_transfer_free(t);
        ct->transfers.clear();
        heap_caps_free(ct->in_data);
        ct->in_data = nullptr;
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
    usb_host_transfer_free(static_cast<usb_transfer_t *>(transfer_handle));
}

void usbipdcpp::Esp32DeviceHandler::send_transfer_data(void *handle, asio::ip::tcp::socket &sock,
                                                       std::size_t offset, std::size_t length,
                                                       std::error_code &ec) {
    {
        std::lock_guard lock(chunked_transfers_mutex_);
        if (auto *ct = try_get_chunked(chunked_transfers_, handle)) {
            if (ct->in_data) {
                asio::write(sock, asio::buffer(
                                    reinterpret_cast<const char *>(ct->in_data) + offset, length), ec);
            }
            else {
                std::size_t remaining = length;
                for (std::size_t i = 0; i < ct->transfers.size() && remaining > 0 && !ec; i++) {
                    auto *trx = ct->transfers[i];
                    std::size_t chunk_off = (i == 0) ? offset : 0;
                    std::size_t chunk_len = std::min(
                            remaining, static_cast<std::size_t>(trx->actual_num_bytes) - chunk_off);
                    if (chunk_len > 0) {
                        asio::write(sock, asio::buffer(
                                            reinterpret_cast<const char *>(trx->data_buffer) + chunk_off, chunk_len),
                                    ec);
                        remaining -= chunk_len;
                    }
                }
            }
            return;
        }
    }
    auto *trx = static_cast<usb_transfer_t *>(handle);
    asio::write(sock, asio::buffer(reinterpret_cast<const char *>(trx->data_buffer) + offset, length), ec);
}

void usbipdcpp::Esp32DeviceHandler::recv_transfer_data(void *handle, asio::ip::tcp::socket &sock,
                                                       std::size_t offset, std::size_t length,
                                                       std::error_code &ec) {
    {
        std::lock_guard lock(chunked_transfers_mutex_);
        if (auto *ct = try_get_chunked(chunked_transfers_, handle)) {
            std::size_t remaining = length;
            for (auto *trx: ct->transfers) {
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
    auto *trx = static_cast<usb_transfer_t *>(handle);
    asio::read(sock, asio::buffer(static_cast<std::uint8_t *>(trx->data_buffer) + offset, length), ec);
}

void usbipdcpp::Esp32DeviceHandler::on_new_connection(Session &current_session, error_code &ec) {
    AbstDeviceHandler::on_new_connection(current_session, ec);
    all_transfer_should_stop = false;
    device_removed_ = false;
    log_heap_diag(TAG);

    // 构建端点 MPS 查找表
    endpoint_mps_map_.clear();
    for (const auto &intf: handle_device.interfaces) {
        for (const auto &ep: intf.endpoints) {
            endpoint_mps_map_[ep.address] = ep.max_packet_size;
        }
    }
    // 控制端点 0 的 MPS
    if (handle_device.ep0_in.max_packet_size > 0) {
        endpoint_mps_map_[0x80] = handle_device.ep0_in.max_packet_size;
    }
    if (handle_device.ep0_out.max_packet_size > 0) {
        endpoint_mps_map_[0x00] = handle_device.ep0_out.max_packet_size;
    }
    for (const auto &[addr, mps] : endpoint_mps_map_) {
        SPDLOG_INFO("endpoint mps: addr={:02x}, mps={}", addr, mps);
    }
}

void usbipdcpp::Esp32DeviceHandler::on_disconnection(error_code &ec) {
    all_transfer_should_stop = true;

    if (device_removed_) [[unlikely]] {
        SPDLOG_WARN("设备已移除，不需要停止传输");
        AbstDeviceHandler::on_disconnection(ec);
        return;
    }

    // 取消所有端点的传输（遍历实际端点，不依赖 tracker 快照）
    for (const auto &intf: handle_device.interfaces) {
        for (const auto &ep: intf.endpoints) {
            cancel_endpoint_all_transfers(ep.address);
        }
    }

    // 等待所有传输完成
    {
        std::unique_lock lock(transfer_complete_mutex_);
        transfer_complete_cv_.wait(lock, [this]() {
            return pending_count_ == 0 && chunked_count_ == 0;
        });
    }

    // Note: callback_args_pool_ is intentionally NOT cleared here. ObjectPool::clear()
    // permanently destroys all preallocated slots and there is no re-init path, so
    // clearing on session disconnect would force every subsequent alloc() in a
    // following session to fall back to heap `new`, defeating the purpose of the pool.
    // The pool's lifetime matches the handler's; its destructor releases the slots.
    AbstDeviceHandler::on_disconnection(ec);
}

void usbipdcpp::Esp32DeviceHandler::handle_unlink_seqnum(std::uint32_t unlink_seqnum, std::uint32_t cmd_seqnum) {
    if (device_removed_) [[unlikely]]
            return;
    SPDLOG_INFO("handle_unlink_seqnum unlink_seqnum:{},cmd_seqnum:{}", unlink_seqnum, cmd_seqnum);

    bool found = false;

    // 分块传输：如果已有分块完成（传输已经实际开始），不 cancel，让剩余分块正常完成。
    // 但必须设 unlinked，让回调发 ret_unlink（带实际结果）替代 ret_submit。
    {
        std::lock_guard lock(chunked_transfers_mutex_);
        auto it = chunked_transfers_.find(unlink_seqnum);
        if (it != chunked_transfers_.end()) {
            auto *ct = it->second;
            // 找到任意未释放的分块获取 cb（分块可能已被提前释放）
            esp32_callback_args *cb = nullptr;
            for (auto *t: ct->transfers) {
                if (t) {
                    cb = static_cast<esp32_callback_args *>(t->context);
                    break;
                }
            }
            if (cb) {
                cb->unlinked = true;
                cb->unlink_cmd_seqnum = cmd_seqnum;
                found = true;
                if (ct->pending_count < static_cast<int>(ct->transfers.size())) {
                    SPDLOG_INFO("handle_unlink_seqnum seqnum={}: {} chunks already done, skip cancel",
                                unlink_seqnum, ct->transfers.size() - ct->pending_count);
                }
                else {
                    cancel_endpoint_all_transfers(ct->ep_address);
                }
            }
            // cb 为空说明所有分块已释放，found 保持 false，走下方 ret_unlink(0)
        }
    }

    // 非分块传输
    if (!found) {
        std::shared_lock lock(transfers_mutex_);
        auto it = transfers_.find(unlink_seqnum);
        if (it != transfers_.end()) {
            auto *cb = it->second;
            cb->unlinked = true;
            cb->unlink_cmd_seqnum = cmd_seqnum;
            found = true;
            lock.unlock();
            cancel_endpoint_all_transfers(static_cast<usb_transfer_t *>(cb->transfer.get())->bEndpointAddress);
        }
    }

    if (!found) {
        // 不在任何表中——传输已完成（回调已入队 RET_SUBMIT 或 RET_UNLINK）。
        // 此时 CMD_UNLINK 已经无关紧要，入队 RET_UNLINK(0) 但不唤醒 sender，
        // 等待下一次 callback 的 wakeup_sender() 顺带发出，或 session 清理时发出。
        SPDLOG_DEBUG("transfer {} 已不在传输表中，入队 ret_unlink {}", unlink_seqnum, cmd_seqnum);
        session->enqueue_ret_unlink(
                UsbIpResponse::UsbIpRetUnlink::create_ret_unlink(cmd_seqnum, 0));
    }
}

void usbipdcpp::Esp32DeviceHandler::receive_urb(
        UsbIpCommand::UsbIpCmdSubmit cmd,
        UsbEndpoint ep,
        std::optional<UsbInterface> interface,
        usbipdcpp::error_code &ec) {

    if (device_removed_) [[unlikely]] {
        ec = make_error_code(ErrorType::NO_DEVICE);
        return;
    }

    auto seqnum = cmd.header.seqnum;
    auto transfer_flags = cmd.transfer_flags;
    auto transfer_buffer_length = cmd.transfer_buffer_length;
    const auto &setup_packet = cmd.setup;
    bool is_control = (ep.attributes == static_cast<std::uint8_t>(EndpointAttributes::Control));

    // 控制传输：特殊请求处理 + 非控制传输需要接口
    if (is_control) [[unlikely]] {
        auto tweak_ret = tweak_special_requests(setup_packet);
        if (tweak_ret >= 0) {
            session->submit_ret_submit(
                    UsbIpResponse::UsbIpRetSubmit::create_ret_submit_ok_without_data(seqnum, transfer_buffer_length));
            return;
        }
    }
    else if (!interface.has_value()) [[unlikely]] {
        SPDLOG_ERROR("非控制传输却不存在目标接口");
        ec = make_error_code(ErrorType::INTERNAL_ERROR);
        return;
    }

    bool is_out = is_control ? setup_packet.is_out() : !ep.is_in();

    // 分块传输
    if (!is_control) {
        bool is_chunked = false;
        {
            std::lock_guard lock(chunked_transfers_mutex_);
            is_chunked = chunked_transfers_.count(seqnum) > 0;
        }
        if (is_chunked) {
            auto *ct = static_cast<ChunkedTransfer *>(cmd.transfer.get());
            ct->seqnum = seqnum;
            ct->is_out = is_out;
            ct->transfer_buffer_length = transfer_buffer_length;
            ct->pending_count = ct->transfers.size();
            ct->total_actual_length = 0;
            ct->worst_status = USB_TRANSFER_STATUS_COMPLETED;
            ct->ep_address = ep.address;
            chunked_count_.fetch_add(1, std::memory_order_release);

            auto *cb = callback_args_pool_.alloc();
            if (!cb) [[unlikely]] cb = new esp32_callback_args{};
            cb->handler = this;
            cb->seqnum = seqnum;
            cb->is_out = is_out;
            cb->chunked = ct;
            cb->transfer = std::move(cmd.transfer);
            cb->transfer_type = (ep.attributes == static_cast<std::uint8_t>(EndpointAttributes::Bulk))
                                    ? USB_TRANSFER_TYPE_BULK
                                    : (ep.attributes == static_cast<std::uint8_t>(EndpointAttributes::Interrupt))
                                    ? USB_TRANSFER_TYPE_INTR
                                    : USB_TRANSFER_TYPE_BULK;

            for (std::size_t i = 0; i < ct->transfers.size(); i++) {
                auto *trx = ct->transfers[i];
                std::uint32_t chunk_bytes = is_out ? trx->num_bytes : trx->data_buffer_size;
                if (!is_out && ep.max_packet_size > 0) {
                    std::uint32_t mps = ep.max_packet_size;
                    if (chunk_bytes % mps != 0) {
                        if (ep.attributes == static_cast<std::uint8_t>(EndpointAttributes::Bulk))
                            chunk_bytes = ((chunk_bytes + mps - 1) / mps) * mps;
                        else
                            chunk_bytes = ((chunk_bytes / mps) + 1) * mps;
                    }
                }
                trx->device_handle = native_handle;
                trx->callback = chunked_transfer_callback;
                trx->context = cb;
                trx->bEndpointAddress = ep.address;
                trx->num_bytes = chunk_bytes;
                trx->flags = get_esp32_transfer_flags(transfer_flags);
                // 中间分块不能发 ZLP，否则设备会误认为传输结束
                if (is_out && ep.attributes == static_cast<std::uint8_t>(EndpointAttributes::Bulk)
                    && i != ct->transfers.size() - 1)
                    trx->flags &= ~USB_TRANSFER_FLAG_ZERO_PACK;

                esp_err_t err = usb_host_transfer_submit(trx);
                if (err != ESP_OK) [[unlikely]] {
                    SPDLOG_ERROR("Chunk submit failed: seqnum={}, err={}", seqnum, esp_err_to_name(err));
                    // 此 chunk 未提交，不计入 pending；已提交的 chunk 会正常回调
                    --ct->pending_count;
                    trx->status = USB_TRANSFER_STATUS_ERROR;
                    trx->actual_num_bytes = 0;
                    if (static_cast<int>(USB_TRANSFER_STATUS_ERROR) > static_cast<int>(ct->worst_status))
                        ct->worst_status = USB_TRANSFER_STATUS_ERROR;
                }
            }
            // 全部提交失败时手动触发清理
            if (ct->pending_count == 0) {
                chunked_count_.fetch_sub(1, std::memory_order_release);
                cb->transfer.reset(); // 触发 free_transfer_handle 清理所有 chunk 并从 map 移除
                session->submit_ret_submit(
                        UsbIpResponse::UsbIpRetSubmit::create_ret_submit_epipe_without_data(seqnum, 0));
                if (!callback_args_pool_.free(cb))
                    delete cb;
            }
            return;
        }
    }

    // num_bytes 和 MPS 对齐
    std::uint32_t num_bytes;
    if (is_control) {
        num_bytes = USB_SETUP_PACKET_SIZE + setup_packet.length;
    }
    else {
        num_bytes = transfer_buffer_length;
        if (!is_out && ep.max_packet_size > 0) {
            std::uint32_t mps = ep.max_packet_size;
            if (num_bytes % mps != 0) {
                if (ep.attributes == static_cast<std::uint8_t>(EndpointAttributes::Bulk))
                    num_bytes = ((num_bytes + mps - 1) / mps) * mps;
                else
                    num_bytes = ((num_bytes / mps) + 1) * mps;
            }
        }
    }

    auto *trx = static_cast<usb_transfer_t *>(cmd.transfer.get());

    // 填充 setup packet 到 buffer 开头
    // OUT 数据已经被 from_socket 写到 buffer + USB_SETUP_PACKET_SIZE 位置
    if (is_control) {
        auto *setup_pkt = reinterpret_cast<usb_setup_packet_t *>(trx->data_buffer);
        setup_pkt->bmRequestType = setup_packet.request_type;
        setup_pkt->bRequest = setup_packet.request;
        setup_pkt->wValue = setup_packet.value;
        setup_pkt->wIndex = setup_packet.index;
        setup_pkt->wLength = setup_packet.length;
    }

    auto *callback_args = callback_args_pool_.alloc();
    if (!callback_args) [[unlikely]] {
        callback_args = new esp32_callback_args{};
    }
    callback_args->handler = this;
    callback_args->seqnum = seqnum;
    callback_args->is_out = is_out;
    callback_args->original_transfer_buffer_length = transfer_buffer_length;
    callback_args->transfer = std::move(cmd.transfer);

    trx->device_handle = native_handle;
    trx->callback = transfer_callback;
    trx->context = callback_args;
    trx->bEndpointAddress = ep.address;
    trx->num_bytes = num_bytes;
    trx->flags = get_esp32_transfer_flags(transfer_flags);

    // 类型特有处理
    if (is_control) {
        callback_args->transfer_type = USB_TRANSFER_TYPE_CTRL;
        SPDLOG_DEBUG("控制传输 {}，ep addr: {:02x}", is_out?"Out":"In", ep.address);
    }
    else if (ep.attributes == static_cast<std::uint8_t>(EndpointAttributes::Bulk)) [[likely]] {
        callback_args->transfer_type = USB_TRANSFER_TYPE_BULK;
        if (is_out) {
            // On bulk OUT, USB_TRANSFER_FLAG_ZERO_PACK asks the host to append a
            // zero-length packet when the payload is an exact multiple of the
            // endpoint MPS, so the device sees a clean end-of-transfer marker.
            trx->flags |= USB_TRANSFER_FLAG_ZERO_PACK;
        }
        SPDLOG_DEBUG("块传输 {}，ep addr: {:02x}, len: {}, num_bytes: {}",
                     is_out?"Out":"In", ep.address, transfer_buffer_length, num_bytes);
    }
    else if (ep.attributes == static_cast<std::uint8_t>(EndpointAttributes::Interrupt)) {
        callback_args->transfer_type = USB_TRANSFER_TYPE_INTR;
        SPDLOG_DEBUG("中断传输 {}，ep addr: {:02x}, len: {}, num_bytes: {}",
                     is_out?"Out":"In", ep.address, transfer_buffer_length, num_bytes);
    }
    else if (ep.attributes == static_cast<std::uint8_t>(EndpointAttributes::Isochronous)) {
        callback_args->transfer_type = USB_TRANSFER_TYPE_ISOCHRONOUS;
        SPDLOG_DEBUG("同步传输 {}，ep addr: {:02x}", is_out?"Out":"In", ep.address);
        // iso_packet_descriptors 已通过 set_iso_descriptor 设置到 trx 中
    }
    else [[unlikely]] {
        SPDLOG_ERROR("端口{:02x}的未知传输类型：{}", ep.address, ep.attributes);
        ec = make_error_code(ErrorType::INVALID_ARG);
        return;
    }

    // 注册到传输表
    {
        std::unique_lock lock(transfers_mutex_);
        transfers_[seqnum] = callback_args;
        pending_count_.fetch_add(1, std::memory_order_release);
    }

    LATENCY_TRACK(session->latency_tracker, seqnum,
                  "Esp32DeviceHandler::receive_urb submit");
    esp_err_t err;
    if (is_control)
        err = usb_host_transfer_submit_control(host_client_handle, trx);
    else
        err = usb_host_transfer_submit(trx);

    if (err != ESP_OK) [[unlikely]] {
        SPDLOG_ERROR("传输失败：seqnum={}, ep={:02x}, type={}, {}, err={}",
                     seqnum, ep.address,
                     is_control ? "ctrl" :
                     ep.attributes == static_cast<std::uint8_t>(EndpointAttributes::Bulk) ? "bulk" :
                     ep.attributes == static_cast<std::uint8_t>(EndpointAttributes::Interrupt) ? "intr" : "iso",
                     is_out ? "out" : "in",
                     esp_err_to_name(err));
        {
            std::unique_lock lock(transfers_mutex_);
            transfers_.erase(seqnum);
            pending_count_.fetch_sub(1, std::memory_order_release);
        }
        callback_args->transfer.reset();
        if (!callback_args_pool_.free(callback_args)) {
            delete callback_args;
        }
        if (err == ESP_ERR_NOT_FOUND) [[unlikely]] {
            device_removed_ = true;
            ec = make_error_code(ErrorType::NO_DEVICE);
        }
        session->submit_ret_submit(
                UsbIpResponse::UsbIpRetSubmit::create_ret_submit_epipe_without_data(seqnum, 0));
    }
}

void usbipdcpp::Esp32DeviceHandler::cancel_all_transfer() {
    const usb_config_desc_t *config_desc;
    ESP_ERROR_CHECK(usb_host_get_active_config_descriptor(native_handle, &config_desc));
    const usb_intf_desc_t *intf = NULL;
    for (int i = 0; i < config_desc->bNumInterfaces; i++) {
        int intf_offset;
        intf = usb_parse_interface_descriptor(config_desc, i, 0, &intf_offset);
        if (!intf)
            continue;

        for (int j = 0; j < intf->bNumEndpoints; j++) {
            int endpoint_offset = intf_offset;
            const usb_ep_desc_t *ep = usb_parse_endpoint_descriptor_by_index(
                    intf, j, config_desc->wTotalLength, &endpoint_offset);
            if (!ep)
                continue;
            cancel_endpoint_all_transfers(ep->bEndpointAddress);
        }
    }
}

void usbipdcpp::Esp32DeviceHandler::cancel_endpoint_all_transfers(uint8_t bEndpointAddress) {
    std::lock_guard lock(endpoint_cancellation_mutex);
    esp_err_t err;
    err = usb_host_endpoint_halt(native_handle, bEndpointAddress);
    if (err != ESP_OK) {
        SPDLOG_ERROR("usb_host_endpoint_halt address {} failed: {}", bEndpointAddress, esp_err_to_name(err));
    }
    err = usb_host_endpoint_flush(native_handle, bEndpointAddress);
    if (err != ESP_OK) {
        SPDLOG_ERROR("usb_host_endpoint_flush address {} failed: {}", bEndpointAddress, esp_err_to_name(err));
    }
    err = usb_host_endpoint_clear(native_handle, bEndpointAddress);
    if (err != ESP_OK) {
        SPDLOG_ERROR("usb_host_endpoint_clear address {} failed: {}", bEndpointAddress, esp_err_to_name(err));
    }
}

esp_err_t usbipdcpp::Esp32DeviceHandler::sync_control_transfer(const SetupPacket &setup_packet) const {
    usb_transfer_t *transfer = nullptr;
    auto err = usb_host_transfer_alloc(USB_SETUP_PACKET_SIZE, 0, &transfer);
    {
        if (err != ESP_OK) {
            SPDLOG_ERROR("无法申请transfer");
            goto error_occurred;
        }
        auto setup_pkt = reinterpret_cast<usb_setup_packet_t *>(transfer->data_buffer);

        setup_pkt->bmRequestType = setup_packet.request_type;
        setup_pkt->bRequest = setup_packet.request;
        setup_pkt->wValue = setup_packet.value;
        setup_pkt->wIndex = setup_packet.index;
        setup_pkt->wLength = setup_packet.length;

        std::binary_semaphore semaphore{0};

        transfer->device_handle = native_handle;
        transfer->callback = [](usb_transfer_t *transfer) {
            if (transfer->status != USB_TRANSFER_STATUS_COMPLETED) {
                SPDLOG_ERROR("sync_control_transfer transfer status is not complete, which is {}",
                             static_cast<std::int32_t>(transfer->status));
            }
            std::binary_semaphore &semaphore = *static_cast<std::binary_semaphore *>(transfer->context);
            semaphore.release();
        };
        transfer->context = &semaphore;
        transfer->bEndpointAddress = setup_packet.calc_ep0_address();
        transfer->num_bytes = USB_SETUP_PACKET_SIZE + setup_packet.length;

        err = usb_host_transfer_submit_control(host_client_handle, transfer);
        if (err != ESP_OK) {
            usb_host_transfer_free(transfer);
            goto error_occurred;
        }
        semaphore.acquire();
        return ESP_OK;
    }

error_occurred:
    return err;
}

int usbipdcpp::Esp32DeviceHandler::tweak_clear_halt_cmd(const SetupPacket &setup_packet) {
    auto target_endp = setup_packet.index;
    SPDLOG_DEBUG("tweak_clear_halt_cmd");

    // 清 ESP32 内部 pipe 状态（HALTED → ACTIVE）
    auto err = usb_host_endpoint_clear(native_handle, target_endp);
    if (err != ESP_OK) [[unlikely]] {
        SPDLOG_ERROR("tweak_clear_halt_cmd usb_host_endpoint_clear error: {}", esp_err_to_name(err));
        return err;
    }
    // 同步发送 CLEAR_FEATURE 给设备，设备端也清除 STALL
    err = sync_control_transfer(setup_packet);
    if (err != ESP_OK) [[unlikely]] {
        SPDLOG_ERROR("tweak_clear_halt_cmd sync_control_transfer error: {}", esp_err_to_name(err));
    }
    SPDLOG_DEBUG("tweak_clear_halt_cmd done: endp {}", target_endp);
    return err; // 返回 0 表示成功，正数表示错误
}

int usbipdcpp::Esp32DeviceHandler::tweak_set_interface_cmd(const SetupPacket &setup_packet) {
    [[maybe_unused]] uint16_t interface = setup_packet.index;
    [[maybe_unused]] uint16_t alternate = setup_packet.value;

    SPDLOG_DEBUG("set_interface: inf {} alt {}", interface, alternate);

    // ESP-IDF's usb_host_lib does not currently support dynamic alt-setting
    // switches. Handling depends on the requested alternate:
    //
    //   * alt 0 — the interface is already at alt 0 after SET_CONFIGURATION,
    //     so a client asking for alt 0 is effectively a no-op. Report
    //     success so standard Linux-kernel enumeration (which issues
    //     SET_INTERFACE(alt=0) for interfaces with multiple alt-settings)
    //     proceeds cleanly.
    //
    //   * alt != 0 — we cannot honor the request. Previously this returned
    //     ESP_OK and the client believed the switch succeeded, which is
    //     silent data corruption for devices that have meaningful
    //     alt-settings (UVC cameras, class-compound audio devices, etc.).
    //     Return an error instead so the client sees the failure.
    if (alternate == 0) {
        SPDLOG_DEBUG("set_interface alt=0 treated as no-op (already at default alt)");
        return ESP_OK;
    }
    SPDLOG_ERROR("set_interface alt={} not supported by ESP-IDF usb_host_lib", alternate);
    ESP_LOGE(TAG, "set_interface alt=%u not supported by ESP-IDF usb_host_lib", alternate);
    return ESP_ERR_NOT_SUPPORTED;
}

int usbipdcpp::Esp32DeviceHandler::tweak_set_configuration_cmd(const SetupPacket &setup_packet) {
    SPDLOG_DEBUG("tweak_set_configuration_cmd");

    // 不可以set_configuration，会device_busy
    // usbipd-libusb 返回 -1，表示不处理这个命令，继续正常提交 transfer
    return -1;
}

int usbipdcpp::Esp32DeviceHandler::tweak_reset_device_cmd(const SetupPacket &setup_packet) {
    SPDLOG_DEBUG("tweak_reset_device_cmd");
    SPDLOG_ERROR("不支持的控制传输reset_device");
    ESP_LOGE(TAG, "不支持的控制传输reset_device");

    // 参考 usbipd-libusb：不执行 reset
    return 0;
}

int usbipdcpp::Esp32DeviceHandler::tweak_special_requests(const SetupPacket &setup_packet) {
    // 返回值：
    // -1: 不需要 tweak，应该提交 transfer
    //  0: tweak 成功，不需要提交 transfer
    // >0: tweak 失败（esp 错误码），不需要提交 transfer
    if (setup_packet.is_clear_halt_cmd()) {
        return tweak_clear_halt_cmd(setup_packet);
    }
    else if (setup_packet.is_set_interface_cmd()) {
        return tweak_set_interface_cmd(setup_packet);
    }
    else if (setup_packet.is_set_configuration_cmd()) {
        return tweak_set_configuration_cmd(setup_packet);
    }
    else if (setup_packet.is_reset_device_cmd()) {
        return tweak_reset_device_cmd(setup_packet);
    }
    SPDLOG_DEBUG("不需要调整包");
    return -1; // 不需要 tweak
}

uint8_t usbipdcpp::Esp32DeviceHandler::get_esp32_transfer_flags(uint32_t in) {
    uint8_t flags = 0;

    if (in & static_cast<std::uint32_t>(TransferFlag::URB_ZERO_PACKET))
        flags |= USB_TRANSFER_FLAG_ZERO_PACK;

    return flags;
}

int usbipdcpp::Esp32DeviceHandler::trxstat2error(usb_transfer_status_t trxstat) {
    switch (trxstat) {
        case USB_TRANSFER_STATUS_COMPLETED:
            return static_cast<int>(UrbStatusType::StatusOK);
        case USB_TRANSFER_STATUS_CANCELED:
            return static_cast<int>(UrbStatusType::StatusECONNRESET);
        case USB_TRANSFER_STATUS_ERROR:
        case USB_TRANSFER_STATUS_STALL:
        case USB_TRANSFER_STATUS_TIMED_OUT:
        case USB_TRANSFER_STATUS_OVERFLOW:
            return static_cast<int>(UrbStatusType::StatusEPIPE);
        case USB_TRANSFER_STATUS_NO_DEVICE:
            return static_cast<int>(UrbStatusType::StatusESHUTDOWN);
        default:
            return static_cast<int>(UrbStatusType::StatusENOENT);
    }
}

usb_transfer_status_t usbipdcpp::Esp32DeviceHandler::error2trxstat(int e) {
    switch (e) {
        case static_cast<int>(UrbStatusType::StatusOK):
            return USB_TRANSFER_STATUS_COMPLETED;
        case static_cast<int>(UrbStatusType::StatusENOENT):
            return USB_TRANSFER_STATUS_ERROR;
        case static_cast<int>(UrbStatusType::StatusECONNRESET):
            return USB_TRANSFER_STATUS_CANCELED;
        case static_cast<int>(UrbStatusType::StatusETIMEDOUT):
            return USB_TRANSFER_STATUS_TIMED_OUT;
        case static_cast<int>(UrbStatusType::StatusEPIPE):
            return USB_TRANSFER_STATUS_STALL;
        case static_cast<int>(UrbStatusType::StatusESHUTDOWN):
            return USB_TRANSFER_STATUS_NO_DEVICE;
        case static_cast<int>(UrbStatusType::StatusEEOVERFLOW):
            return USB_TRANSFER_STATUS_OVERFLOW;
        default:
            return USB_TRANSFER_STATUS_ERROR;
    }
}

void usbipdcpp::Esp32DeviceHandler::chunked_transfer_callback(usb_transfer_t *trx) {
    auto *cb = static_cast<esp32_callback_args *>(trx->context);
    auto *ct = cb->chunked;
    auto *handler = cb->handler;

    LATENCY_TRACK(handler->session->latency_tracker, cb->seqnum,
                  "Esp32DeviceHandler::chunked_transfer_callback调用");

    // 如果断连了，直接清理并返回（不发送响应）
    if (handler->all_transfer_should_stop) [[unlikely]] {
        bool is_last = (--ct->pending_count == 0);
        if (is_last) {
            handler->chunked_count_.fetch_sub(1, std::memory_order_release);
            cb->transfer.reset(); // 触发 free_transfer_handle，清理所有 chunk 并从 map 移除
            if (!handler->callback_args_pool_.free(cb))
                delete cb;
            handler->transfer_complete_cv_.notify_one();
        }
        return;
    }

    // status 检查
    switch (trx->status) {
        case USB_TRANSFER_STATUS_COMPLETED:
            if (trx == ct->transfers[0])
                ct->transfer_started = true;
            break;
        case USB_TRANSFER_STATUS_ERROR:
            SPDLOG_ERROR("chunked transfer error on endpoint {}", trx->bEndpointAddress);
            break;
        case USB_TRANSFER_STATUS_CANCELED: {
            // 持有 chunked_transfers_mutex_，使判断 cb->unlinked 与重提交原子化，
            // 防止 handle_unlink_seqnum 在判断与重提交之间设置 unlinked 并取消端点,
            // 导致重提交的 chunk 逃过取消。
            // transfer_started 为真表示已有数据成功传输，即使 cb->unlinked 也应重提交。
            // STALL 触发的 cancel 不重提交
            bool has_error = (static_cast<int>(ct->worst_status) == static_cast<int>(USB_TRANSFER_STATUS_STALL) ||
                              static_cast<int>(ct->worst_status) == static_cast<int>(USB_TRANSFER_STATUS_NO_DEVICE));
            std::lock_guard ck_lock(handler->chunked_transfers_mutex_);
            if (!has_error && (!cb->unlinked || ct->transfer_started)) {
                trx->status = USB_TRANSFER_STATUS_COMPLETED;
                esp_err_t err;
                {
                    std::shared_lock ep_lock(handler->endpoint_cancellation_mutex);
                    err = usb_host_transfer_submit(trx);
                }
                if (err != ESP_OK) {
                    SPDLOG_ERROR("chunked seqnum为{}的传输重新提交失败：{}", cb->seqnum, esp_err_to_name(err));
                    trx->status = USB_TRANSFER_STATUS_ERROR;
                    trx->actual_num_bytes = 0;
                }
                else {
                    return;
                }
            }
            else {
                auto chunk_idx = std::find(ct->transfers.begin(), ct->transfers.end(), trx) - ct->transfers.begin();
                SPDLOG_INFO("chunk {}/{} seqnum {} canceled on endpoint {}",
                            chunk_idx + 1, ct->transfers.size(), cb->seqnum, trx->bEndpointAddress);
            }
            break;
        }
        case USB_TRANSFER_STATUS_STALL:
            SPDLOG_ERROR("chunked endpoint {} is stalled", trx->bEndpointAddress);
            handler->cancel_endpoint_all_transfers(trx->bEndpointAddress);
            break;
        case USB_TRANSFER_STATUS_NO_DEVICE:
            handler->device_removed_ = true;
            SPDLOG_INFO("chunked device removed?");
            break;
        default:
            SPDLOG_WARN("chunked urb completion with unknown status {}", static_cast<int>(trx->status));
            break;
    }
    auto chunk_idx = std::find(ct->transfers.begin(), ct->transfers.end(), trx) - ct->transfers.begin();
    SPDLOG_DEBUG("chunk {}/{} seqnum {} done, status={}, {} bytes",
                 chunk_idx + 1, ct->transfers.size(), cb->seqnum,
                 static_cast<int>(trx->status), trx->actual_num_bytes);

    // 跟踪最差状态
    if (static_cast<int>(trx->status) > static_cast<int>(ct->worst_status))
        ct->worst_status = trx->status;

    // 立即释放 DMA transfer：OUT 数据已发给设备，IN 数据已拷入 in_data
    if (cb->is_out) {
        ct->total_actual_length += trx->actual_num_bytes;
        ct->transfers[chunk_idx] = nullptr;
        usb_host_transfer_free(trx);
    }
    else if (ct->in_data && trx->status == USB_TRANSFER_STATUS_COMPLETED) {
        // 以 total_actual_length 当前值作为写入偏移，保证连续无缝隙
        std::size_t offset = ct->total_actual_length.load();
        memcpy(ct->in_data + offset, trx->data_buffer, trx->actual_num_bytes);
        ct->total_actual_length += trx->actual_num_bytes;
        ct->transfers[chunk_idx] = nullptr;
        usb_host_transfer_free(trx);
    }
    else {
        ct->total_actual_length += trx->actual_num_bytes;
    }

    if (--ct->pending_count > 0)
        return; // 还有未完成的 chunk

    // ========== 所有 chunk 完成 ==========
    auto seqnum = ct->seqnum;
    auto worst = ct->worst_status;
    bool is_out = ct->is_out;

    // 计算 actual_length：成功传输的字节数
    std::uint32_t actual_length = static_cast<std::uint32_t>(ct->total_actual_length.load());

    // 在锁内检查 unlinked、入队响应——原子完成。
    // map 由 free_transfer_handle 统一擦除，不在此处擦，避免 free_transfer_handle
    // 查找不到 ct 而将 ChunkedTransfer* 误当 usb_transfer_t* 释放。
    bool need_reset_transfer = false;
    {
        std::lock_guard lock(handler->chunked_transfers_mutex_);
        if (cb->unlinked) {
            SPDLOG_INFO("CHUNKED DONE seqnum={} {} UNLINK unlink_cmd={} worst_status={} actual_length={}",
                        seqnum, is_out ? "OUT" : "IN", cb->unlink_cmd_seqnum,
                        static_cast<int>(worst), actual_length);
            LATENCY_TRACK_END_MSG(handler->session->latency_tracker, cb->unlink_cmd_seqnum, "被unlink");
            handler->session->enqueue_ret_unlink(
                    UsbIpResponse::UsbIpRetUnlink::create_ret_unlink(
                            cb->unlink_cmd_seqnum, handler->trxstat2error(worst)));
            need_reset_transfer = true;
        }
        else {
            UsbIpResponse::UsbIpRetSubmit ret;
            if (is_out) {
                ret = UsbIpResponse::UsbIpRetSubmit::create_ret_submit_with_status_and_no_data(
                        seqnum, handler->trxstat2error(worst), actual_length);
                need_reset_transfer = true;
            }
            else {
                ret = UsbIpResponse::UsbIpRetSubmit::create_ret_submit(
                        seqnum, handler->trxstat2error(worst),
                        actual_length, 0, 0, std::move(cb->transfer));
            }
            SPDLOG_INFO("CHUNKED DONE seqnum={} {} RET_SUBMIT status={} actual_length={}",
                        seqnum, is_out ? "OUT" : "IN",
                        handler->trxstat2error(worst), actual_length);
            LATENCY_TRACK(handler->session->latency_tracker, seqnum,
                          "Esp32DeviceHandler::chunked_transfer_callback submit_ret_submit");
            handler->session->enqueue_ret_submit(std::move(ret));
        }
        handler->chunked_count_.fetch_sub(1, std::memory_order_release);
    }
    // 在锁外 reset，避免 free_transfer_handle 内拿 chunked_transfers_mutex_ 死锁
    if (need_reset_transfer) {
        cb->transfer.reset();
    }
    handler->session->wakeup_sender();

    if (!handler->callback_args_pool_.free(cb))
        delete cb;

    // 无条件通知，覆盖正常路径与 on_disconnection 的竞态。
    // CV 谓词确保只有最后一个 callback 真正唤醒 wait。
    handler->transfer_complete_cv_.notify_one();
}

void usbipdcpp::Esp32DeviceHandler::transfer_callback(usb_transfer_t *trx) {
    auto *cb = static_cast<esp32_callback_args *>(trx->context);
    auto *handler = cb->handler;

    LATENCY_TRACK(handler->session->latency_tracker, cb->seqnum,
                  "Esp32DeviceHandler::transfer_callback调用");

    // 如果断连了，直接清理并返回（不发送响应）
    if (handler->all_transfer_should_stop) [[unlikely]] {
        {
            std::unique_lock lock(handler->transfers_mutex_);
            handler->transfers_.erase(cb->seqnum);
            handler->pending_count_.fetch_sub(1, std::memory_order_release);
        }
        cb->transfer.reset();
        if (!handler->callback_args_pool_.free(cb))
            delete cb;
        // 无条件通知，覆盖与 on_disconnection 的竞态。
        // CV 谓词确保只有最后一个 callback 真正唤醒 wait。
        handler->transfer_complete_cv_.notify_one();
        return;
    }

    // 持有 transfers_mutex_ 进行 CANCELED 判断，使判断与重提交之间原子化，
    // 防止 handle_unlink_seqnum 在判断与重提交之间设置 unlinked 并取消端点。
    {
        std::unique_lock lock(handler->transfers_mutex_);

        // status 检查
        switch (trx->status) {
            case USB_TRANSFER_STATUS_COMPLETED:
                break;
            case USB_TRANSFER_STATUS_ERROR:
                SPDLOG_ERROR("transfer error on endpoint {}", trx->bEndpointAddress);
                break;
            case USB_TRANSFER_STATUS_CANCELED: {
                // ESP32 取消端点时会连带取消该端点所有 transfer
                // 未被标记 unlinked 的需要重新提交（持有 transfers_mutex_，与 handle_unlink_seqnum 互斥）
                if (!cb->unlinked) {
                    trx->status = USB_TRANSFER_STATUS_COMPLETED;
                    esp_err_t err;
                    {
                        std::shared_lock ep_lock(handler->endpoint_cancellation_mutex);
                        if (cb->transfer_type == USB_TRANSFER_TYPE_CTRL) {
                            err = usb_host_transfer_submit_control(handler->host_client_handle, trx);
                        }
                        else {
                            err = usb_host_transfer_submit(trx);
                        }
                    }
                    if (err != ESP_OK) {
                        SPDLOG_ERROR("seqnum为{}的传输重新提交失败：{}", cb->seqnum, esp_err_to_name(err));
                        handler->transfers_.erase(cb->seqnum);
                        handler->pending_count_.fetch_sub(1, std::memory_order_release);
                        lock.unlock();
                        handler->session->submit_ret_submit(
                                UsbIpResponse::UsbIpRetSubmit::create_ret_submit_epipe_without_data(cb->seqnum, 0));
                        cb->transfer.reset();
                        if (!handler->callback_args_pool_.free(cb))
                            delete cb;
                    }
                    return;
                }
                SPDLOG_INFO("transfer seqnum {} canceled on endpoint {}", cb->seqnum, trx->bEndpointAddress);
                break;
            }
            case USB_TRANSFER_STATUS_STALL:
                SPDLOG_ERROR("endpoint {} is stalled", trx->bEndpointAddress);
                break;
            case USB_TRANSFER_STATUS_NO_DEVICE:
                handler->device_removed_ = true;
                SPDLOG_INFO("device removed?");
                break;
            default:
                SPDLOG_WARN("urb completion with unknown status {}", static_cast<int>(trx->status));
                break;
        }
    }
    SPDLOG_DEBUG("esp32传输了{}个字节", trx->actual_num_bytes);

    // 计算 actual_length
    std::uint32_t actual_length = trx->actual_num_bytes;
    bool is_control = (trx->bEndpointAddress & 0x7F) == 0;
    if (!cb->is_out && is_control) {
        if (actual_length > USB_SETUP_PACKET_SIZE)
            actual_length -= USB_SETUP_PACKET_SIZE;
        else
            actual_length = 0;
    }
    if (trx->num_isoc_packets > 0 && !cb->is_out) {
        std::uint32_t iso_actual = 0;
        for (int i = 0; i < trx->num_isoc_packets; i++)
            iso_actual += trx->isoc_packet_desc[i].actual_num_bytes;
        actual_length = iso_actual;
    }

    // 在锁内检查 unlinked、入队响应、移除追踪——原子完成。
    // handle_unlink_seqnum 若在此之前执行，会看到 map 中有此 entry 并设置 unlinked；
    // 若在此之后，则看不到，自行入队 RET_UNLINK(0)。
    {
        std::unique_lock lock(handler->transfers_mutex_);
        bool unlinked = cb->unlinked;
        std::uint32_t unlink_cmd_seqnum = cb->unlink_cmd_seqnum;
        handler->transfers_.erase(cb->seqnum);
        handler->pending_count_.fetch_sub(1, std::memory_order_release);

        if (unlinked) {
            LATENCY_TRACK_END_MSG(handler->session->latency_tracker, unlink_cmd_seqnum, "被unlink");
            handler->session->enqueue_ret_unlink(
                    UsbIpResponse::UsbIpRetUnlink::create_ret_unlink(
                            unlink_cmd_seqnum, trxstat2error(trx->status)));
            cb->transfer.reset();
        }
        else {
            UsbIpResponse::UsbIpRetSubmit ret;
            if (cb->is_out) {
                ret = UsbIpResponse::UsbIpRetSubmit::create_ret_submit_with_status_and_no_data(
                        cb->seqnum, trxstat2error(trx->status), actual_length);
                cb->transfer.reset();
            }
            else {
                ret = UsbIpResponse::UsbIpRetSubmit::create_ret_submit(
                        cb->seqnum, trxstat2error(trx->status),
                        actual_length, 0, trx->num_isoc_packets,
                        std::move(cb->transfer));
            }
            SPDLOG_DEBUG("esp32传输actual_length为{}个字节", actual_length);
            LATENCY_TRACK(handler->session->latency_tracker, cb->seqnum,
                          "Esp32DeviceHandler::transfer_callback submit_ret_submit");
            handler->session->enqueue_ret_submit(std::move(ret));
        }
    }
    handler->session->wakeup_sender();

    if (!handler->callback_args_pool_.free(cb))
        delete cb;

    // 无条件通知，覆盖正常路径与 on_disconnection 的竞态。
    // CV 谓词确保只有最后一个 callback 真正唤醒 wait。
    handler->transfer_complete_cv_.notify_one();
}
