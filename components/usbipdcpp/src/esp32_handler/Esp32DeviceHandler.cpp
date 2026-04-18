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

void usbipdcpp::Esp32DeviceHandler::on_new_connection(Session &current_session, error_code &ec) {
    AbstDeviceHandler::on_new_connection(current_session, ec);
    all_transfer_should_stop = false;
    device_removed_ = false;
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

    callback_args_pool_.clear();
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
        const SetupPacket &setup_packet, data_type &&transfer_buffer,
        [[maybe_unused]] std::error_code &ec) {

    if (device_removed_) [[unlikely]] {
        ec = make_error_code(ErrorType::NO_DEVICE);
        return;
    }

    auto tweak_ret = tweak_special_requests(setup_packet);
    if (tweak_ret < 0) [[likely]] {
        // 不需要 tweak，提交 transfer
        SPDLOG_DEBUG("控制传输 {}，ep addr: {:02x}", ep.direction() == UsbEndpoint::Direction::Out?"Out":"In", ep.address);

        usb_transfer_t *transfer = nullptr;
        auto err = usb_host_transfer_alloc(USB_SETUP_PACKET_SIZE + transfer_buffer_length, 0, &transfer);
        if (err != ESP_OK) [[unlikely]] {
            SPDLOG_ERROR("无法申请transfer");
            session->submit_ret_submit(
                    UsbIpResponse::UsbIpRetSubmit::create_ret_submit_epipe_without_data(seqnum, 0));
            return;
        }

        if (setup_packet.is_out()) {
            memcpy(transfer->data_buffer + USB_SETUP_PACKET_SIZE, transfer_buffer.data(), transfer_buffer.size());
        }

        auto *callback_args = callback_args_pool_.alloc();
        if (!callback_args) [[unlikely]] {
            callback_args = new esp32_callback_args{};
        }
        callback_args->handler = this;
        callback_args->seqnum = seqnum;
        callback_args->transfer_type = USB_TRANSFER_TYPE_CTRL;
        callback_args->is_out = setup_packet.is_out();
        callback_args->original_transfer_buffer_length = transfer_buffer_length;

        auto *setup_pkt = reinterpret_cast<usb_setup_packet_t *>(transfer->data_buffer);
        setup_pkt->bmRequestType = setup_packet.request_type;
        setup_pkt->bRequest = setup_packet.request;
        setup_pkt->wValue = setup_packet.value;
        setup_pkt->wIndex = setup_packet.index;
        setup_pkt->wLength = setup_packet.length;

        transfer->device_handle = native_handle;
        transfer->callback = transfer_callback;
        transfer->context = callback_args;
        transfer->bEndpointAddress = ep.address;
        transfer->num_bytes = USB_SETUP_PACKET_SIZE + setup_packet.length;
        transfer->flags = get_esp32_transfer_flags(transfer_flags);

        transfer_tracker_.register_transfer(seqnum, transfer, ep.address);

        LATENCY_TRACK(session->latency_tracker, seqnum,
                      "Esp32DeviceHandler::handle_control_urb usb_host_transfer_submit_control");
        err = usb_host_transfer_submit_control(host_client_handle, transfer);

        if (err != ESP_OK) [[unlikely]] {
            SPDLOG_ERROR("控制传输给设备失败：{}", esp_err_to_name(err));
            transfer_tracker_.remove(seqnum);
            if (!callback_args_pool_.free(callback_args)) {
                delete callback_args;
            }
            usb_host_transfer_free(transfer);
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
    session->submit_ret_submit(
            UsbIpResponse::UsbIpRetSubmit::create_ret_submit_ok_without_data(seqnum, 0));
}

void usbipdcpp::Esp32DeviceHandler::handle_bulk_transfer(std::uint32_t seqnum, const UsbEndpoint &ep,
                                                         UsbInterface &interface,
                                                         std::uint32_t transfer_flags,
                                                         std::uint32_t transfer_buffer_length,
                                                         data_type &&transfer_buffer,
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
    usb_transfer_t *transfer = nullptr;
    auto err = usb_host_transfer_alloc(aligned_length, 0, &transfer);
    if (err != ESP_OK) [[unlikely]] {
        SPDLOG_ERROR("无法申请transfer");
        session->submit_ret_submit(
                UsbIpResponse::UsbIpRetSubmit::create_ret_submit_epipe_without_data(seqnum, 0));
        return;
    }
    auto *callback_args = callback_args_pool_.alloc();
    if (!callback_args) [[unlikely]] {
        callback_args = new esp32_callback_args{};
    }
    callback_args->handler = this;
    callback_args->seqnum = seqnum;
    callback_args->transfer_type = USB_TRANSFER_TYPE_BULK;
    callback_args->is_out = is_out;
    callback_args->original_transfer_buffer_length = transfer_buffer_length;

    if (is_out) {
        memcpy(transfer->data_buffer, transfer_buffer.data(), transfer_buffer.size());
    }

    transfer->device_handle = native_handle;
    transfer->callback = transfer_callback;
    transfer->context = callback_args;
    transfer->bEndpointAddress = ep.address;
    transfer->num_bytes = aligned_length;
    transfer->flags = get_esp32_transfer_flags(transfer_flags);
    if (is_out) {
        transfer->flags &= USB_TRANSFER_FLAG_ZERO_PACK;
    }

    transfer_tracker_.register_transfer(seqnum, transfer, ep.address);

    LATENCY_TRACK(session->latency_tracker, seqnum,
                  "Esp32DeviceHandler::handle_bulk_transfer usb_host_transfer_submit");
    err = usb_host_transfer_submit(transfer);

    if (err != ESP_OK) [[unlikely]] {
        SPDLOG_ERROR("块传输失败，{}", esp_err_to_name(err));
        transfer_tracker_.remove(seqnum);
        if (!callback_args_pool_.free(callback_args)) {
            delete callback_args;
        }
        usb_host_transfer_free(transfer);
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
                                                              data_type &&transfer_buffer,
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
    usb_transfer_t *transfer = nullptr;
    auto err = usb_host_transfer_alloc(aligned_length, 0, &transfer);
    if (err != ESP_OK) [[unlikely]] {
        SPDLOG_ERROR("无法申请transfer");
        session->submit_ret_submit(
                UsbIpResponse::UsbIpRetSubmit::create_ret_submit_epipe_without_data(seqnum, 0));
        return;
    }

    if (is_out) {
        assert(transfer_buffer_length == transfer_buffer.size());
    }

    auto *callback_args = callback_args_pool_.alloc();
    if (!callback_args) [[unlikely]] {
        callback_args = new esp32_callback_args{};
    }
    callback_args->handler = this;
    callback_args->seqnum = seqnum;
    callback_args->transfer_type = USB_TRANSFER_TYPE_INTR;
    callback_args->is_out = is_out;
    callback_args->original_transfer_buffer_length = transfer_buffer_length;

    if (is_out) {
        memcpy(transfer->data_buffer, transfer_buffer.data(), transfer_buffer.size());
    }

    transfer->device_handle = native_handle;
    transfer->callback = transfer_callback;
    transfer->context = callback_args;
    transfer->bEndpointAddress = ep.address;
    transfer->num_bytes = aligned_length;
    transfer->flags = get_esp32_transfer_flags(transfer_flags);

    transfer_tracker_.register_transfer(seqnum, transfer, ep.address);

    err = usb_host_transfer_submit(transfer);

    if (err != ESP_OK) [[unlikely]] {
        SPDLOG_ERROR("中断传输失败，{}", esp_err_to_name(err));
        transfer_tracker_.remove(seqnum);
        if (!callback_args_pool_.free(callback_args)) {
            delete callback_args;
        }
        usb_host_transfer_free(transfer);
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
        data_type &&transfer_buffer,
        const std::vector<UsbIpIsoPacketDescriptor> &
        iso_packet_descriptors,
        [[maybe_unused]] std::error_code &ec) {
    if (device_removed_) [[unlikely]] {
        ec = make_error_code(ErrorType::NO_DEVICE);
        return;
    }

    bool is_out = !ep.is_in();
    SPDLOG_DEBUG("同步传输 {}，ep addr: {:02x}", is_out?"Out":"In", ep.address);

    auto num_iso_packets = static_cast<int>(iso_packet_descriptors.size());
    usb_transfer_t *transfer = nullptr;
    auto err = usb_host_transfer_alloc(transfer_buffer_length, num_iso_packets, &transfer);
    if (err != ESP_OK) [[unlikely]] {
        SPDLOG_ERROR("无法申请transfer");
        session->submit_ret_submit(
                UsbIpResponse::UsbIpRetSubmit::create_ret_submit_epipe_without_data(seqnum, 0));
        return;
    }

    auto *callback_args = callback_args_pool_.alloc();
    if (!callback_args) [[unlikely]] {
        callback_args = new esp32_callback_args{};
    }
    callback_args->handler = this;
    callback_args->seqnum = seqnum;
    callback_args->transfer_type = USB_TRANSFER_TYPE_ISOCHRONOUS;
    callback_args->is_out = is_out;
    callback_args->original_transfer_buffer_length = transfer_buffer_length;

    if (is_out) {
        memcpy(transfer->data_buffer, transfer_buffer.data(), transfer_buffer.size());
    }

    transfer->device_handle = native_handle;
    transfer->callback = transfer_callback;
    transfer->context = callback_args;
    transfer->bEndpointAddress = ep.address;
    transfer->num_bytes = transfer_buffer_length;
    transfer->flags = get_esp32_transfer_flags(transfer_flags);

    for (std::size_t i = 0; i < iso_packet_descriptors.size(); i++) {
        auto &iso_desc_i = transfer->isoc_packet_desc[i];
        iso_desc_i.status = error2trxstat(iso_packet_descriptors[i].status);
        iso_desc_i.actual_num_bytes = iso_packet_descriptors[i].actual_length;
        iso_desc_i.num_bytes = iso_packet_descriptors[i].length;
    }

    transfer_tracker_.register_transfer(seqnum, transfer, ep.address);

    err = usb_host_transfer_submit(transfer);
    if (err != ESP_OK) [[unlikely]] {
        SPDLOG_ERROR("同步传输失败，{}", esp_err_to_name(err));
        transfer_tracker_.remove(seqnum);
        if (!callback_args_pool_.free(callback_args)) {
            delete callback_args;
        }
        usb_host_transfer_free(transfer);
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
    SPDLOG_ERROR("不支持的控制传输 set_interface");
    ESP_LOGE(TAG, "不支持的控制传输 set_interface");

    // ESP-IDF 暂不支持动态切换 alternate setting
    return ESP_OK;
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
        if (!callback_arg->handler->callback_args_pool_.free(callback_arg)) {
            delete callback_arg;
        }
        usb_host_transfer_free(trx);
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
                    // 重新提交失败，需要从 tracker 移除，发送错误响应并清理
                    callback_arg->handler->transfer_tracker_.remove(callback_arg->seqnum);
                    callback_arg->handler->session->submit_session_response(
                            SessionResponse::with_cleanup(
                                    UsbIpResponse::UsbIpRetSubmit::create_ret_submit_epipe_without_data(
                                            callback_arg->seqnum, 0),
                                    callback_arg,
                                    trx,
                                    &Esp32DeviceHandler::cleanup_transfer_resources
                                    ));
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

    if (!is_unlinked) [[likely]] {
        // 默认为这个值，对于控制传输和等时传输可能改变
        size_t actual_data_len = trx->actual_num_bytes;
        std::vector<UsbIpIsoPacketDescriptor> iso_packet_descriptors{};
        // 发送 ret_submit
        // OUT 传输不需要发送数据回客户端，只发送 header
        // IN 传输需要发送 header + 数据
        data_type response_buffer;
        if (!callback_arg->is_out) {
            if (callback_arg->transfer_type == USB_TRANSFER_TYPE_CTRL) {
                actual_data_len = trx->actual_num_bytes > USB_SETUP_PACKET_SIZE
                                      ? trx->actual_num_bytes - USB_SETUP_PACKET_SIZE
                                      : 0;
                // 用原始请求长度限制返回数据
                actual_data_len = std::min(actual_data_len,
                                           static_cast<size_t>(callback_arg->original_transfer_buffer_length));
                if (actual_data_len > 0) {
                    response_buffer.assign(
                            trx->data_buffer + USB_SETUP_PACKET_SIZE,
                            trx->data_buffer + USB_SETUP_PACKET_SIZE + actual_data_len
                            );
                }
            }
            else if (callback_arg->transfer_type == USB_TRANSFER_TYPE_ISOCHRONOUS) {
                iso_packet_descriptors.resize(trx->num_isoc_packets);
                size_t iso_actual_length = 0;
                for (int i = 0; i < trx->num_isoc_packets; i++) {
                    iso_actual_length += trx->isoc_packet_desc[i].actual_num_bytes;
                }
                // 用原始请求长度限制返回数据
                iso_actual_length = std::min(iso_actual_length,
                                             static_cast<size_t>(callback_arg->original_transfer_buffer_length));
                response_buffer.resize(iso_actual_length, 0);
                size_t received_data_offset = 0;
                size_t trx_buffer_offset = 0;
                for (int i = 0; i < trx->num_isoc_packets; i++) {
                    auto &iso_packet = trx->isoc_packet_desc[i];
                    std::memcpy(response_buffer.data() + received_data_offset, trx->data_buffer + trx_buffer_offset,
                                iso_packet.actual_num_bytes);
                    iso_packet_descriptors[i].offset = received_data_offset;
                    iso_packet_descriptors[i].length = iso_packet.actual_num_bytes;
                    iso_packet_descriptors[i].actual_length = iso_packet.actual_num_bytes;
                    iso_packet_descriptors[i].status = trxstat2error(iso_packet.status);

                    //记录在内存中的长度以便发送时遍历每一小节
                    iso_packet_descriptors[i].length_in_transfer_buffer_only_for_send = iso_packet.actual_num_bytes;

                    received_data_offset += iso_packet.actual_num_bytes;
                    trx_buffer_offset += iso_packet.num_bytes;
                }

                actual_data_len = iso_actual_length;
            }
            else {
                // 用原始请求长度限制返回数据
                actual_data_len = std::min(actual_data_len,
                                           static_cast<size_t>(callback_arg->original_transfer_buffer_length));
                if (actual_data_len > 0) {
                    response_buffer.assign(
                            trx->data_buffer,
                            trx->data_buffer + actual_data_len
                            );
                }
            }
        }


        callback_arg->handler->session->submit_ret_submit(
                UsbIpResponse::UsbIpRetSubmit::create_ret_submit(
                        callback_arg->seqnum,
                        trxstat2error(trx->status),
                        actual_data_len,
                        0,
                        trx->num_isoc_packets,
                        std::move(response_buffer),
                        std::move(iso_packet_descriptors)
                        )
                );
    }
    else {
        // unlink 情况：发送 ret_unlink
        LATENCY_TRACK_END_MSG(callback_arg->handler->session->latency_tracker, unlink_cmd_seqnum, "被unlink");

        callback_arg->handler->session->submit_session_response(
                SessionResponse::with_cleanup(
                        UsbIpResponse::UsbIpRetUnlink::create_ret_unlink(
                                unlink_cmd_seqnum,
                                trxstat2error(trx->status)
                                ),
                        callback_arg,
                        trx,
                        &Esp32DeviceHandler::cleanup_transfer_resources
                        )
                );
    }
}

void usbipdcpp::Esp32DeviceHandler::cleanup_transfer_resources(void *context, void *transfer) {
    auto *callback_arg = static_cast<esp32_callback_args *>(context);
    auto *trx = static_cast<usb_transfer_t *>(transfer);

    // 清理资源
    if (!callback_arg->handler->callback_args_pool_.free(callback_arg)) {
        delete callback_arg;
    }
    usb_host_transfer_free(trx);
}
