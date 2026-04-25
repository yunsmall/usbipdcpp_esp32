#include "esp32_handler/Esp32DeviceHandler.h"

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
    ESP_ERROR_CHECK(usb_host_device_info(native_handle, &device_info));
}

usbipdcpp::Esp32DeviceHandler::~Esp32DeviceHandler() = default;

// ========== transfer_handle 操作实现 ==========

void* usbipdcpp::Esp32DeviceHandler::alloc_transfer_handle(std::size_t buffer_length, int num_iso_packets, const UsbIpHeaderBasic& header, const SetupPacket& setup_packet) {
    // 计算实际需要分配的大小
    std::size_t actual_buffer_length = buffer_length;
    bool is_control = (header.ep == 0);
    bool is_in = (header.direction == static_cast<std::uint32_t>(UsbIpDirection::In));

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
        SPDLOG_ERROR("usb_host_transfer_alloc failed: {}", esp_err_to_name(err));
        return nullptr;
    }
    return transfer;
}

void* usbipdcpp::Esp32DeviceHandler::get_transfer_buffer(void* transfer_handle) {
    auto* trx = static_cast<usb_transfer_t*>(transfer_handle);
    return trx->data_buffer;
}

std::size_t usbipdcpp::Esp32DeviceHandler::get_actual_length(void* transfer_handle) {
    auto* trx = static_cast<usb_transfer_t*>(transfer_handle);
    return trx->actual_num_bytes;
}

std::size_t usbipdcpp::Esp32DeviceHandler::get_read_data_offset(void* transfer_handle) {
    auto* trx = static_cast<usb_transfer_t*>(transfer_handle);
    // 控制传输使用端点 0（地址 0x00 或 0x80）
    if ((trx->bEndpointAddress & 0x7F) == 0) {
        return USB_SETUP_PACKET_SIZE;  // 8
    }
    return 0;
}

std::size_t usbipdcpp::Esp32DeviceHandler::get_write_data_offset(const UsbIpHeaderBasic& header) {
    // 控制传输 (ep == 0) 需要跳过 setup packet
    if (header.ep == 0) {
        return USB_SETUP_PACKET_SIZE;
    }
    return 0;
}

usbipdcpp::UsbIpIsoPacketDescriptor usbipdcpp::Esp32DeviceHandler::get_iso_descriptor(void* transfer_handle, int index) {
    auto* trx = static_cast<usb_transfer_t*>(transfer_handle);
    auto& iso = trx->isoc_packet_desc[index];
    return UsbIpIsoPacketDescriptor{
        .offset = 0,  // 需要调用方计算
        .length = static_cast<std::uint32_t>(iso.num_bytes),
        .actual_length = static_cast<std::uint32_t>(iso.actual_num_bytes),
        .status = static_cast<std::uint32_t>(trxstat2error(iso.status)),
        .length_in_transfer_buffer_only_for_send = static_cast<std::uint32_t>(iso.num_bytes)
    };
}

void usbipdcpp::Esp32DeviceHandler::set_iso_descriptor(void* transfer_handle, int index, const UsbIpIsoPacketDescriptor& desc) {
    auto* trx = static_cast<usb_transfer_t*>(transfer_handle);
    auto& iso = trx->isoc_packet_desc[index];
    iso.status = error2trxstat(desc.status);
    iso.actual_num_bytes = desc.actual_length;
    iso.num_bytes = desc.length;
}

void usbipdcpp::Esp32DeviceHandler::free_transfer_handle(void* transfer_handle) {
    usb_host_transfer_free(static_cast<usb_transfer_t*>(transfer_handle));
}

void usbipdcpp::Esp32DeviceHandler::on_new_connection(Session &current_session, error_code &ec) {
    AbstDeviceHandler::on_new_connection(current_session, ec);
    all_transfer_should_stop = false;
    device_removed_ = false;

    // 构建端点 MPS 查找表
    endpoint_mps_map_.clear();
    for (const auto &intf : handle_device.interfaces) {
        for (const auto &ep : intf.endpoints) {
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
}

void usbipdcpp::Esp32DeviceHandler::on_disconnection(error_code &ec) {
    all_transfer_should_stop = true;

    if (device_removed_) [[unlikely]] {
        SPDLOG_WARN("设备已移除，不需要停止传输");
        AbstDeviceHandler::on_disconnection(ec);
        return;
    }

    // 取消所有传输
    auto transfers = transfer_tracker_.get_all_transfers();
    for (auto &info: transfers) {
        cancel_endpoint_all_transfers(info.endpoint);
    }

    // 等待所有传输完成
    {
        std::unique_lock lock(transfer_complete_mutex_);
        transfer_complete_cv_.wait(lock, [this]() {
            return transfer_tracker_.concurrent_count() == 0;
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

    usb_transfer_t *transfer_to_cancel = nullptr;

    transfer_tracker_.with_transfer(unlink_seqnum, [&](auto *info) {
        if (info) {
            // transfer 还在进行中，在锁保护下标记为 unlinking
            info->is_unlinked = true;
            info->unlink_cmd_seqnum = cmd_seqnum;
            transfer_to_cancel = info->transfer;
        }
    });

    if (transfer_to_cancel) [[likely]] {
        cancel_endpoint_all_transfers(transfer_to_cancel->bEndpointAddress);
    }
    else {
        // 传输已完成，立即发送 ret_unlink
        SPDLOG_DEBUG("transfer {} 已完成，立即发送 ret_unlink {}", unlink_seqnum, cmd_seqnum);
        session->submit_ret_unlink(
                UsbIpResponse::UsbIpRetUnlink::create_ret_unlink(cmd_seqnum, 0));
    }
}

void usbipdcpp::Esp32DeviceHandler::handle_control_urb(
        std::uint32_t seqnum,
        const UsbEndpoint &ep,
        std::uint32_t transfer_flags,
        std::uint32_t transfer_buffer_length,
        const SetupPacket &setup_packet, TransferHandle transfer,
        [[maybe_unused]] std::error_code &ec) {

    if (device_removed_) [[unlikely]] {
        ec = make_error_code(ErrorType::NO_DEVICE);
        return;
    }

    auto tweak_ret = tweak_special_requests(setup_packet);
    if (tweak_ret < 0) [[likely]] {
        // 不需要 tweak，提交 transfer
        SPDLOG_DEBUG("控制传输 {}，ep addr: {:02x}", ep.direction() == UsbEndpoint::Direction::Out?"Out":"In", ep.address);

        auto* trx = static_cast<usb_transfer_t*>(transfer.get());

        // 填充 setup packet 到 buffer 开头
        // OUT 数据已经被 from_socket 写到 buffer + USB_SETUP_PACKET_SIZE 位置
        auto* setup_pkt = reinterpret_cast<usb_setup_packet_t *>(trx->data_buffer);
        setup_pkt->bmRequestType = setup_packet.request_type;
        setup_pkt->bRequest = setup_packet.request;
        setup_pkt->wValue = setup_packet.value;
        setup_pkt->wIndex = setup_packet.index;
        setup_pkt->wLength = setup_packet.length;

        auto *callback_args = callback_args_pool_.alloc();
        if (!callback_args) [[unlikely]] {
            callback_args = new esp32_callback_args{};
        }
        callback_args->handler = this;
        callback_args->seqnum = seqnum;
        callback_args->transfer_type = USB_TRANSFER_TYPE_CTRL;
        callback_args->is_out = setup_packet.is_out();
        callback_args->original_transfer_buffer_length = transfer_buffer_length;
        callback_args->transfer = std::move(transfer);  // 转移所有权

        trx->device_handle = native_handle;
        trx->callback = transfer_callback;
        trx->context = callback_args;
        trx->bEndpointAddress = ep.address;
        trx->num_bytes = USB_SETUP_PACKET_SIZE + setup_packet.length;
        trx->flags = get_esp32_transfer_flags(transfer_flags);

        transfer_tracker_.register_transfer(seqnum, trx, ep.address);

        LATENCY_TRACK(session->latency_tracker, seqnum,
                      "Esp32DeviceHandler::handle_control_urb usb_host_transfer_submit_control");
        esp_err_t err = usb_host_transfer_submit_control(host_client_handle, trx);

        if (err != ESP_OK) [[unlikely]] {
            SPDLOG_ERROR("控制传输给设备失败：{}", esp_err_to_name(err));
            transfer_tracker_.remove(seqnum);
            callback_args->transfer.reset();  // 提前释放
            if (!callback_args_pool_.free(callback_args)) {
                delete callback_args;
            }
            session->submit_ret_submit(
                    UsbIpResponse::UsbIpRetSubmit::create_ret_submit_epipe_without_data(seqnum, 0));
            if (err == ESP_ERR_NOT_FOUND) [[unlikely]] {
                device_removed_ = true;
                ec = make_error_code(ErrorType::NO_DEVICE);
            }
        }
        return;
    }
    // tweak 成功或失败，都不提交 transfer，发送成功响应
    // transfer 析构时会自动释放
    session->submit_ret_submit(
            UsbIpResponse::UsbIpRetSubmit::create_ret_submit_ok_without_data(seqnum, transfer_buffer_length));
}

void usbipdcpp::Esp32DeviceHandler::handle_bulk_transfer(std::uint32_t seqnum, const UsbEndpoint &ep,
                                                         UsbInterface &interface,
                                                         std::uint32_t transfer_flags,
                                                         std::uint32_t transfer_buffer_length,
                                                         TransferHandle transfer,
                                                         [[maybe_unused]] std::error_code &ec) {
    if (device_removed_) [[unlikely]] {
        ec = make_error_code(ErrorType::NO_DEVICE);
        return;
    }

    bool is_out = !ep.is_in();

    // ESP32 要求批量 IN 传输的 num_bytes 必须是 MPS 的整数倍
    std::uint32_t aligned_length = transfer_buffer_length;
    if (!is_out && ep.max_packet_size > 0) {
        std::uint32_t mps = ep.max_packet_size;
        if (aligned_length % mps != 0) {
            aligned_length = ((aligned_length + mps - 1) / mps) * mps;
        }
    }

    LATENCY_TRACK(session->latency_tracker, seqnum, "Esp32DeviceHandler::handle_bulk_transfer进来");

    SPDLOG_DEBUG("块传输 {}，ep addr: {:02x}, len: {}, aligned: {}",
                 is_out?"Out":"In", ep.address, transfer_buffer_length, aligned_length);

    auto* trx = static_cast<usb_transfer_t*>(transfer.get());

    auto *callback_args = callback_args_pool_.alloc();
    if (!callback_args) [[unlikely]] {
        callback_args = new esp32_callback_args{};
    }
    callback_args->handler = this;
    callback_args->seqnum = seqnum;
    callback_args->transfer_type = USB_TRANSFER_TYPE_BULK;
    callback_args->is_out = is_out;
    callback_args->original_transfer_buffer_length = transfer_buffer_length;
    callback_args->transfer = std::move(transfer);  // 转移所有权

    trx->device_handle = native_handle;
    trx->callback = transfer_callback;
    trx->context = callback_args;
    trx->bEndpointAddress = ep.address;
    trx->num_bytes = aligned_length;
    trx->flags = get_esp32_transfer_flags(transfer_flags);
    if (is_out) {
        // On bulk OUT, USB_TRANSFER_FLAG_ZERO_PACK asks the host to append a
        // zero-length packet when the payload is an exact multiple of the
        // endpoint MPS, so the device sees a clean end-of-transfer marker.
        trx->flags |= USB_TRANSFER_FLAG_ZERO_PACK;
    }

    transfer_tracker_.register_transfer(seqnum, trx, ep.address);

    LATENCY_TRACK(session->latency_tracker, seqnum,
                  "Esp32DeviceHandler::handle_bulk_transfer usb_host_transfer_submit");
    esp_err_t err = usb_host_transfer_submit(trx);

    if (err != ESP_OK) [[unlikely]] {
        SPDLOG_ERROR("块传输失败，{}", esp_err_to_name(err));
        transfer_tracker_.remove(seqnum);
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

void usbipdcpp::Esp32DeviceHandler::handle_interrupt_transfer(std::uint32_t seqnum,
                                                              const UsbEndpoint &ep,
                                                              UsbInterface &interface,
                                                              std::uint32_t transfer_flags,
                                                              std::uint32_t transfer_buffer_length,
                                                              TransferHandle transfer,
                                                              [[maybe_unused]] std::error_code &ec) {
    if (device_removed_) [[unlikely]] {
        ec = make_error_code(ErrorType::NO_DEVICE);
        return;
    }

    bool is_out = !ep.is_in();

    // ESP32 要求中断 IN 传输的 num_bytes 必须是 MPS 的整数倍
    std::uint32_t aligned_length = transfer_buffer_length;
    if (!is_out && ep.max_packet_size > 0) {
        std::uint32_t mps = ep.max_packet_size;
        if (aligned_length % mps != 0) {
            aligned_length = ((aligned_length / mps) + 1) * mps;
        }
    }

    SPDLOG_DEBUG("中断传输 {}，ep addr: {:02x}, len: {}, aligned: {}",
                 is_out?"Out":"In", ep.address, transfer_buffer_length, aligned_length);

    auto* trx = static_cast<usb_transfer_t*>(transfer.get());

    auto *callback_args = callback_args_pool_.alloc();
    if (!callback_args) [[unlikely]] {
        callback_args = new esp32_callback_args{};
    }
    callback_args->handler = this;
    callback_args->seqnum = seqnum;
    callback_args->transfer_type = USB_TRANSFER_TYPE_INTR;
    callback_args->is_out = is_out;
    callback_args->original_transfer_buffer_length = transfer_buffer_length;
    callback_args->transfer = std::move(transfer);  // 转移所有权

    trx->device_handle = native_handle;
    trx->callback = transfer_callback;
    trx->context = callback_args;
    trx->bEndpointAddress = ep.address;
    trx->num_bytes = aligned_length;
    trx->flags = get_esp32_transfer_flags(transfer_flags);

    transfer_tracker_.register_transfer(seqnum, trx, ep.address);

    esp_err_t err = usb_host_transfer_submit(trx);

    if (err != ESP_OK) [[unlikely]] {
        SPDLOG_ERROR("中断传输失败，{}", esp_err_to_name(err));
        transfer_tracker_.remove(seqnum);
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

void usbipdcpp::Esp32DeviceHandler::handle_isochronous_transfer(
        std::uint32_t seqnum,
        const UsbEndpoint &ep,
        UsbInterface &interface,
        std::uint32_t transfer_flags,
        std::uint32_t transfer_buffer_length,
        TransferHandle transfer,
        int num_iso_packets,
        [[maybe_unused]] std::error_code &ec) {
    if (device_removed_) [[unlikely]] {
        ec = make_error_code(ErrorType::NO_DEVICE);
        return;
    }

    bool is_out = !ep.is_in();
    SPDLOG_DEBUG("同步传输 {}，ep addr: {:02x}", is_out?"Out":"In", ep.address);

    auto* trx = static_cast<usb_transfer_t*>(transfer.get());

    auto *callback_args = callback_args_pool_.alloc();
    if (!callback_args) [[unlikely]] {
        callback_args = new esp32_callback_args{};
    }
    callback_args->handler = this;
    callback_args->seqnum = seqnum;
    callback_args->transfer_type = USB_TRANSFER_TYPE_ISOCHRONOUS;
    callback_args->is_out = is_out;
    callback_args->original_transfer_buffer_length = transfer_buffer_length;
    callback_args->transfer = std::move(transfer);  // 转移所有权

    trx->device_handle = native_handle;
    trx->callback = transfer_callback;
    trx->context = callback_args;
    trx->bEndpointAddress = ep.address;
    trx->num_bytes = transfer_buffer_length;
    trx->flags = get_esp32_transfer_flags(transfer_flags);

    // iso_packet_descriptors 已通过 set_iso_descriptor 设置到 trx 中

    transfer_tracker_.register_transfer(seqnum, trx, ep.address);

    esp_err_t err = usb_host_transfer_submit(trx);
    if (err != ESP_OK) [[unlikely]] {
        SPDLOG_ERROR("同步传输失败，{}", esp_err_to_name(err));
        transfer_tracker_.remove(seqnum);
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

    auto err = usb_host_endpoint_clear(native_handle, target_endp);
    if (err != ESP_OK) [[unlikely]] {
        SPDLOG_ERROR("tweak_clear_halt_cmd usb_host_endpoint_clear error: {}", esp_err_to_name(err));
    }
    else {
        SPDLOG_DEBUG("tweak_clear_halt_cmd done: endp {}", target_endp);
    }
    return err; // 返回 0 表示成功，正数表示错误
}

int usbipdcpp::Esp32DeviceHandler::tweak_set_interface_cmd(const SetupPacket &setup_packet) {
    uint16_t interface = setup_packet.index;
    uint16_t alternate = setup_packet.value;

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

void usbipdcpp::Esp32DeviceHandler::transfer_callback(usb_transfer_t *trx) {
    auto *callback_arg = static_cast<esp32_callback_args *>(trx->context);

    LATENCY_TRACK(callback_arg->handler->session->latency_tracker, callback_arg->seqnum,
                  "Esp32DeviceHandler::transfer_callback调用");

    // 先用 with_transfer 只读取 is_unlinked 信息，不移除
    // 因为 ESP32 取消端点时会取消该端点所有 transfer，可能需要重新提交
    bool is_unlinked = false;
    std::uint32_t unlink_cmd_seqnum = 0;

    callback_arg->handler->transfer_tracker_.with_transfer(callback_arg->seqnum, [&](auto *info) {
        if (info) {
            is_unlinked = info->is_unlinked;
            unlink_cmd_seqnum = info->unlink_cmd_seqnum;
        }
    });

    // 如果断连了，直接清理并返回（不发送响应）
    if (callback_arg->handler->all_transfer_should_stop) [[unlikely]] {
        callback_arg->handler->transfer_tracker_.remove(callback_arg->seqnum);
        if (callback_arg->handler->transfer_tracker_.concurrent_count() == 0) {
            std::lock_guard lock(callback_arg->handler->transfer_complete_mutex_);
            callback_arg->handler->transfer_complete_cv_.notify_one();
        }
        // 清理并返回
        callback_arg->transfer.reset();
        if (!callback_arg->handler->callback_args_pool_.free(callback_arg)) {
            delete callback_arg;
        }
        return;
    }

    // status 检查
    switch (trx->status) {
        case USB_TRANSFER_STATUS_COMPLETED:
            /* OK */
            break;
        case USB_TRANSFER_STATUS_ERROR:
            SPDLOG_ERROR("transfer error on endpoint {}", trx->bEndpointAddress);
            break;
        case USB_TRANSFER_STATUS_CANCELED: {
            // ESP32 USB Host 取消端点传输时会取消该端点上所有传输
            // 如果不是被 unlink 的，需要重新提交，不移除 tracker 记录
            if (!is_unlinked) {
                // 取消的不是自己，重新提交自己
                trx->status = USB_TRANSFER_STATUS_COMPLETED;
                esp_err_t err;
                {
                    std::shared_lock lock(callback_arg->handler->endpoint_cancellation_mutex);
                    if (callback_arg->transfer_type == USB_TRANSFER_TYPE_CTRL) {
                        SPDLOG_TRACE("尝试重新提交控制传输");
                        err = usb_host_transfer_submit_control(callback_arg->handler->host_client_handle, trx);
                    }
                    else {
                        SPDLOG_TRACE("尝试重新提交非控制传输");
                        err = usb_host_transfer_submit(trx);
                    }
                }
                if (err != ESP_OK) {
                    SPDLOG_ERROR("seqnum为{}的传输重新提交失败：{}", callback_arg->seqnum, esp_err_to_name(err));
                    // 重新提交失败，需要从 tracker 移除
                    callback_arg->handler->transfer_tracker_.remove(callback_arg->seqnum);
                    callback_arg->handler->session->submit_ret_submit(
                            UsbIpResponse::UsbIpRetSubmit::create_ret_submit_epipe_without_data(
                                    callback_arg->seqnum, 0));
                    callback_arg->transfer.reset();
                    if (!callback_arg->handler->callback_args_pool_.free(callback_arg)) {
                        delete callback_arg;
                    }
                }
                // 重新提交成功，传输仍在 tracker 中，直接返回
                return;
            }
            SPDLOG_INFO("transfer canceled on endpoint {}", trx->bEndpointAddress);
            break;
        }
        case USB_TRANSFER_STATUS_STALL:
            SPDLOG_ERROR("endpoint {} is stalled", trx->bEndpointAddress);
            break;
        case USB_TRANSFER_STATUS_NO_DEVICE:
            callback_arg->handler->device_removed_ = true;
            SPDLOG_INFO("device removed?");
            break;
        default:
            SPDLOG_WARN("urb completion with unknown status {}", static_cast<int>(trx->status));
            break;
    }
    SPDLOG_DEBUG("esp32传输了{}个字节", trx->actual_num_bytes);

    // 发送响应之前，从 tracker 中移除传输
    callback_arg->handler->transfer_tracker_.remove(callback_arg->seqnum);

    // 计算 actual_length
    std::uint32_t actual_length = trx->actual_num_bytes;

    // 控制传输 IN：减去 setup packet 大小
    bool is_control = (trx->bEndpointAddress & 0x7F) == 0;
    if (!callback_arg->is_out && is_control) {
        if (actual_length > USB_SETUP_PACKET_SIZE) {
            actual_length -= USB_SETUP_PACKET_SIZE;
        } else {
            actual_length = 0;
        }
    }

    // ISO 传输：计算所有 iso packet 的 actual_length 之和
    if (trx->num_isoc_packets > 0 && !callback_arg->is_out) {
        std::uint32_t iso_actual = 0;
        for (int i = 0; i < trx->num_isoc_packets; i++) {
            iso_actual += trx->isoc_packet_desc[i].actual_num_bytes;
        }
        actual_length = iso_actual;
    }

    if (!is_unlinked) [[likely]] {
        // 发送 ret_submit
        UsbIpResponse::UsbIpRetSubmit ret;
        if (callback_arg->is_out) {
            // OUT 传输：无数据阶段
            ret = UsbIpResponse::UsbIpRetSubmit::create_ret_submit_with_status_and_no_data(
                    callback_arg->seqnum,
                    trxstat2error(trx->status),
                    actual_length
                    );
            // OUT 传输：释放 transfer
            callback_arg->transfer.reset();
        }
        else {
            // IN 传输：有数据，转移所有权
            ret = UsbIpResponse::UsbIpRetSubmit::create_ret_submit(
                    callback_arg->seqnum,
                    trxstat2error(trx->status),
                    actual_length,
                    0,  // start_frame
                    trx->num_isoc_packets,
                    std::move(callback_arg->transfer)  // 转移所有权
                    );
        }

        SPDLOG_DEBUG("esp32传输actual_length为{}个字节", actual_length);

        LATENCY_TRACK(callback_arg->handler->session->latency_tracker, callback_arg->seqnum,
                      "Esp32DeviceHandler::transfer_callback submit_ret_submit");
        callback_arg->handler->session->submit_ret_submit(std::move(ret));
    }
    else {
        // unlink 情况：发送 ret_unlink
        LATENCY_TRACK_END_MSG(callback_arg->handler->session->latency_tracker, unlink_cmd_seqnum, "被unlink");

        callback_arg->handler->session->submit_ret_unlink(
                UsbIpResponse::UsbIpRetUnlink::create_ret_unlink(
                        unlink_cmd_seqnum,
                        trxstat2error(trx->status)
                        )
                );
        // unlink 情况：释放 transfer
        callback_arg->transfer.reset();
    }

    // 释放 callback_arg
    // TransferHandle 已被转移或重置，可以安全归还到池
    if (!callback_arg->handler->callback_args_pool_.free(callback_arg)) {
        delete callback_arg;
    }
}
