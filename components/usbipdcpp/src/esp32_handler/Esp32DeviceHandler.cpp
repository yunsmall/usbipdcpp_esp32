#include "esp32_handler/Esp32DeviceHandler.h"

#include <esp_log.h>

#include "Session.h"
#include "protocol.h"
#include "SetupPacket.h"
#include "constant.h"
#include "endpoint.h"

const char *usbipdcpp::Esp32DeviceHandler::TAG = "Esp32DeviceHandler";

usbipdcpp::Esp32DeviceHandler::Esp32DeviceHandler(UsbDevice &handle_device, usb_device_handle_t native_handle,
                                                  usb_host_client_handle_t host_client_handle):
    DeviceHandlerBase(handle_device), native_handle(native_handle), host_client_handle(host_client_handle) {
    ESP_ERROR_CHECK(usb_host_device_info(native_handle, &device_info));
}

usbipdcpp::Esp32DeviceHandler::~Esp32DeviceHandler() {
}

void usbipdcpp::Esp32DeviceHandler::on_new_connection(Session &current_session, error_code &ec) {
    session = &current_session;
    all_transfer_should_stop = false;
}

void usbipdcpp::Esp32DeviceHandler::on_disconnection(error_code &ec) {
    all_transfer_should_stop = true;
    if (!has_device) {
        SPDLOG_WARN("没有设备，不需要停止传输");
        session = nullptr;
        return;
    }
    cancel_all_transfer();
    spdlog::info("成功取消所有传输");
    transferring_data.clear();
    session = nullptr;
}

void usbipdcpp::Esp32DeviceHandler::handle_unlink_seqnum(std::uint32_t seqnum) {
    if (!has_device) {
        //设备已经没了不可以再取消传输
        return;
    }
    cancel_all_transfer();
}

void usbipdcpp::Esp32DeviceHandler::handle_control_urb(
        std::uint32_t seqnum,
        const UsbEndpoint &ep,
        std::uint32_t transfer_flags,
        std::uint32_t transfer_buffer_length,
        const SetupPacket &setup_packet, const data_type &req,
        [[maybe_unused]] std::error_code &ec) {
    if (!has_device) {
        ec = make_error_code(ErrorType::NO_DEVICE);
        return;
    }
    //auto tweaked = -1;
    auto tweaked = tweak_special_requests(setup_packet);
    //尝试执行特殊操作再对usb进行控制
    if (!tweaked) {
        SPDLOG_DEBUG("控制传输 {}，ep addr: {:02x}", ep.direction() == UsbEndpoint::Direction::Out?"Out":"In", ep.address);

        usb_transfer_t *transfer = nullptr;
        auto err = usb_host_transfer_alloc(USB_SETUP_PACKET_SIZE + transfer_buffer_length, 0, &transfer);
        {
            if (err != ESP_OK) {
                SPDLOG_ERROR("无法申请transfer");
                ec = make_error_code(ErrorType::TRANSFER_ERROR);
                return;
            }
            if (setup_packet.is_out()) {
                memcpy(transfer->data_buffer + USB_SETUP_PACKET_SIZE, req.data(), req.size());
            }
            auto *callback_args = new esp32_callback_args{
                    .handler = *this,
                    .seqnum = seqnum,
                    .transfer_type = USB_TRANSFER_TYPE_CTRL,
                    .is_out = setup_packet.is_out()
            };
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

            //将usbio transder flag转换成libusb的flags
            transfer->flags = get_esp32_transfer_flags(transfer_flags);

            {
                std::lock_guard lock(transferring_data_mutex);
                transferring_data[seqnum] = transfer;
            }

            err = usb_host_transfer_submit_control(host_client_handle, transfer);
            if (err != ESP_OK) {
                SPDLOG_ERROR("transfer提交失败");
                usb_host_transfer_free(transfer);
                delete callback_args;
                goto error_occurred;
            }
        }
        return;
    error_occurred:
        SPDLOG_ERROR("控制传输失败:{}", esp_err_to_name(err));
        session->submit_ret_submit(
                UsbIpResponse::UsbIpRetSubmit::create_ret_submit_epipe_without_data(seqnum));
        return;
    }
    else {
        SPDLOG_INFO("拦截了控制包：{}", seqnum);
        //SPDLOG_INFO("拦截了包: {}", setup_packet.to_string());
        //返回空包
        session->submit_ret_submit(UsbIpResponse::UsbIpRetSubmit::create_ret_submit(
                seqnum,
                static_cast<std::uint32_t>(UrbStatusType::StatusOK),
                0,
                0,
                {},
                {}
                ));
        return;
    }
}

void usbipdcpp::Esp32DeviceHandler::handle_bulk_transfer(std::uint32_t seqnum, const UsbEndpoint &ep,
                                                         UsbInterface &interface,
                                                         std::uint32_t transfer_flags,
                                                         std::uint32_t transfer_buffer_length,
                                                         const data_type &out_data,
                                                         [[maybe_unused]] std::error_code &ec) {
    if (!has_device) {
        ec = make_error_code(ErrorType::NO_DEVICE);
        return;
    }

    bool is_out = !ep.is_in();

    SPDLOG_DEBUG("块传输 {}，ep addr: {:02x}", is_out?"Out":"In", ep.address);
    usb_transfer_t *transfer = nullptr;
    auto err = usb_host_transfer_alloc(transfer_buffer_length, 0, &transfer);
    {
        if (err != ESP_OK) {
            SPDLOG_ERROR("无法申请transfer");
            ec = make_error_code(ErrorType::TRANSFER_ERROR);
            return;
        }
        if (is_out) {
            memcpy(transfer->data_buffer, out_data.data(), out_data.size());
        }
        auto *callback_args = new esp32_callback_args{
                .handler = *this,
                .seqnum = seqnum,
                .transfer_type = USB_TRANSFER_TYPE_BULK,
                .is_out = is_out
        };
        transfer->device_handle = native_handle;
        transfer->callback = transfer_callback;
        transfer->context = callback_args;
        transfer->bEndpointAddress = ep.address;
        transfer->num_bytes = transfer_buffer_length;
        transfer->flags = get_esp32_transfer_flags(transfer_flags);
        if (is_out) {
            transfer->flags &= USB_TRANSFER_FLAG_ZERO_PACK;
        }

        {
            std::lock_guard lock(transferring_data_mutex);
            transferring_data[seqnum] = transfer;
        }

        err = usb_host_transfer_submit(transfer);

        if (err != ESP_OK) {
            SPDLOG_ERROR("transfer提交失败");
            usb_host_transfer_free(transfer);
            delete callback_args;
            goto error_occurred;
        }
    }
    return;
error_occurred:
    SPDLOG_ERROR("块传输失败，{}", esp_err_to_name(err));
    session->submit_ret_submit(
            UsbIpResponse::UsbIpRetSubmit::create_ret_submit_epipe_without_data(seqnum));
}

void usbipdcpp::Esp32DeviceHandler::handle_interrupt_transfer(std::uint32_t seqnum,
                                                              const UsbEndpoint &ep,
                                                              UsbInterface &interface,
                                                              std::uint32_t transfer_flags,
                                                              std::uint32_t transfer_buffer_length,
                                                              const data_type &out_data,
                                                              [[maybe_unused]] std::error_code &ec) {
    if (!has_device) {
        ec = make_error_code(ErrorType::NO_DEVICE);
        return;
    }

    bool is_out = !ep.is_in();

    SPDLOG_DEBUG("中断传输 {}，ep addr: {:02x}", is_out?"Out":"In", ep.address);
    usb_transfer_t *transfer = nullptr;
    auto err = usb_host_transfer_alloc(transfer_buffer_length, 0, &transfer);
    {
        if (err != ESP_OK) {
            SPDLOG_ERROR("无法申请transfer");
            return;
        }
        if (is_out) {
            memcpy(transfer->data_buffer, out_data.data(), out_data.size());
        }
        auto *callback_args = new esp32_callback_args{
                .handler = *this,
                .seqnum = seqnum,
                .transfer_type = USB_TRANSFER_TYPE_INTR,
                .is_out = is_out
        };
        transfer->device_handle = native_handle;
        transfer->callback = transfer_callback;
        transfer->context = callback_args;
        transfer->bEndpointAddress = ep.address;
        transfer->num_bytes = transfer_buffer_length;
        transfer->flags = get_esp32_transfer_flags(transfer_flags);

        {
            std::lock_guard lock(transferring_data_mutex);
            transferring_data[seqnum] = transfer;
        }

        err = usb_host_transfer_submit(transfer);

        if (err != ESP_OK) {
            SPDLOG_ERROR("transfer提交失败");
            usb_host_transfer_free(transfer);
            delete callback_args;
            goto error_occurred;
        }
    }
    return;
error_occurred:
    SPDLOG_ERROR("中断传输失败，{}", esp_err_to_name(err));
    //不认为是错误，让服务器重置
    // ec = make_error_code(ErrorType::TRANSFER_ERROR);
    session->submit_ret_submit(
            UsbIpResponse::UsbIpRetSubmit::create_ret_submit_epipe_without_data(seqnum));
}

void usbipdcpp::Esp32DeviceHandler::handle_isochronous_transfer(
        std::uint32_t seqnum,
        const UsbEndpoint &ep,
        UsbInterface &interface,
        std::uint32_t transfer_flags,
        std::uint32_t transfer_buffer_length,
        const data_type &req,
        const std::vector<UsbIpIsoPacketDescriptor> &
        iso_packet_descriptors,
        [[maybe_unused]] std::error_code &ec) {
    if (!has_device) {
        ec = make_error_code(ErrorType::NO_DEVICE);
        return;
    }

    bool is_out = !ep.is_in();
    SPDLOG_DEBUG("同步传输 {}，ep addr: {:02x}", is_out?"Out":"In", ep.address);

    usb_transfer_t *transfer = nullptr;
    auto err = usb_host_transfer_alloc(transfer_buffer_length, iso_packet_descriptors.size(), &transfer);
    {
        if (err != ESP_OK) {
            SPDLOG_ERROR("无法申请transfer");
            ec = make_error_code(ErrorType::TRANSFER_ERROR);
            return;
        }
        if (is_out) {
            memcpy(transfer->data_buffer, req.data(), req.size());
        }
        auto *callback_args = new esp32_callback_args{
                .handler = *this,
                .seqnum = seqnum,
                .transfer_type = USB_TRANSFER_TYPE_ISOCHRONOUS,
                .is_out = is_out
        };
        transfer->device_handle = native_handle;
        transfer->callback = transfer_callback;
        transfer->context = callback_args;
        transfer->bEndpointAddress = ep.address;
        transfer->num_bytes = transfer_buffer_length;

        transfer->flags = get_esp32_transfer_flags(transfer_flags);

        for (std::size_t i = 0; i < iso_packet_descriptors.size(); i++) {
            auto &libusb_iso_desc_i = transfer->isoc_packet_desc[i];
            /* ignore iso->offset; */
            libusb_iso_desc_i.status = error2trxstat(iso_packet_descriptors[i].status);
            libusb_iso_desc_i.actual_num_bytes = iso_packet_descriptors[i].actual_length;
            libusb_iso_desc_i.num_bytes = iso_packet_descriptors[i].length;
        }

        transfer->flags = get_esp32_transfer_flags(transfer_flags);

        {
            std::lock_guard lock(transferring_data_mutex);
            transferring_data[seqnum] = transfer;
        }

        err = usb_host_transfer_submit(transfer);
        if (err < 0) {
            SPDLOG_ERROR("transfer提交失败");
            usb_host_transfer_free(transfer);
            delete callback_args;
            goto error_occurred;
        }
    }
    return;
error_occurred:
    SPDLOG_ERROR("同步传输失败，{}", esp_err_to_name(err));
    session->submit_ret_submit(
            UsbIpResponse::UsbIpRetSubmit::create_ret_submit_epipe_without_data(seqnum));
}

void usbipdcpp::Esp32DeviceHandler::cancel_all_transfer() {

    // 0号端口不支持
    // cancel_endpoint_all_transfers(native_handle, 0x00);
    // cancel_endpoint_all_transfers(native_handle, 0x80);

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
            // 清除当前端点的所有传输
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
            //不管传输的结果
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
        //等待回调函数结束
        semaphore.acquire();
        return ESP_OK;
    }

error_occurred:
    // SPDLOG_ERROR("sync_control_transfer error occured : {}", esp_err_to_name(err));
    return err;
}

esp_err_t usbipdcpp::Esp32DeviceHandler::tweak_clear_halt_cmd(const SetupPacket &setup_packet) {
    auto target_endp = setup_packet.index;
    SPDLOG_DEBUG("tweak_clear_halt_cmd");

    auto err = usb_host_endpoint_clear(native_handle, target_endp);
    if (err != ESP_OK) {
        SPDLOG_ERROR("tweak_clear_halt_cmd usb_host_endpoint_clear error: {}", esp_err_to_name(err));
        return err;
    }
    return ESP_OK;
}

esp_err_t usbipdcpp::Esp32DeviceHandler::tweak_set_interface_cmd(const SetupPacket &setup_packet) {
    uint16_t alternate = setup_packet.value;
    uint16_t interface = setup_packet.index;

    SPDLOG_DEBUG("set_interface: inf {} alt {}",
                 interface, alternate);
    SPDLOG_ERROR("不支持的控制传输 set_interface");
    ESP_LOGE(TAG, "不支持的控制传输 set_interface");

    // auto err = sync_control_transfer(setup_packet);
    // if (err != ESP_OK) {
    //     SPDLOG_ERROR("error occurred in tweak_set_interface_cmd:{}", esp_err_to_name(err));
    //     return err;
    // }
    return ESP_OK;
}

esp_err_t usbipdcpp::Esp32DeviceHandler::tweak_set_configuration_cmd(const SetupPacket &setup_packet) {
    SPDLOG_DEBUG("tweak_set_configuration_cmd");

    //不可以set_configuration，会device_busy
    //就当执行过了
    return ESP_OK;
}

esp_err_t usbipdcpp::Esp32DeviceHandler::tweak_reset_device_cmd(const SetupPacket &setup_packet) {
    SPDLOG_DEBUG("tweak_reset_device_cmd");
    SPDLOG_ERROR("不支持的控制传输reset_device");
    ESP_LOGE(TAG, "不支持的控制传输reset_device");
    // auto err = sync_control_transfer(setup_packet);
    // if (err != ESP_OK) {
    //     SPDLOG_ERROR("error occurred in tweak_reset_device_cmd:{}", esp_err_to_name(err));
    //     return err;
    // }
    return ESP_OK;
}

bool usbipdcpp::Esp32DeviceHandler::tweak_special_requests(const SetupPacket &setup_packet) {
    bool tweaked = false;
    if (setup_packet.is_clear_halt_cmd()) {
        tweaked = tweak_clear_halt_cmd(setup_packet) == ESP_OK;
    }
    else if (setup_packet.is_set_interface_cmd()) {
        tweaked = tweak_set_interface_cmd(setup_packet) == ESP_OK;
    }
    else if (setup_packet.is_set_configuration_cmd()) {
        tweaked = tweak_set_configuration_cmd(setup_packet) == ESP_OK;
    }
    else if (setup_packet.is_reset_device_cmd()) {
        tweaked = tweak_reset_device_cmd(setup_packet) == ESP_OK;
    }
    SPDLOG_DEBUG("不需要调整包");
    return tweaked;
}

uint8_t usbipdcpp::Esp32DeviceHandler::get_esp32_transfer_flags(uint32_t in) {
    uint8_t flags = 0;

    // if (in & static_cast<std::uint32_t>(TransferFlag::URB_SHORT_NOT_OK))
    //     USB_TRANSFER_FLAG_ZERO_PACK
    //     flags |= LIBUSB_TRANSFER_SHORT_NOT_OK;
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
        case static_cast<int>(UrbStatusType::StatusEEOVERFLOW): //EOVERFLOW
            return USB_TRANSFER_STATUS_OVERFLOW;
        default:
            return USB_TRANSFER_STATUS_ERROR;
    }
}

void usbipdcpp::Esp32DeviceHandler::transfer_callback(usb_transfer_t *trx) {
    auto &callback_arg = *static_cast<esp32_callback_args *>(
        trx->context);
    if (callback_arg.handler.all_transfer_should_stop)
        return;

    //调了回调则当前包并未在发送，因此只要调了回调就先将其删了
    {
        std::lock_guard lock(callback_arg.handler.transferring_data_mutex);
        callback_arg.handler.transferring_data.erase(callback_arg.seqnum);
    }

    auto unlink_found = callback_arg.handler.session->get_unlink_seqnum(callback_arg.seqnum);
    // std::error_code ec;
    switch (trx->status) {
        case USB_TRANSFER_STATUS_COMPLETED:
            /* OK */
            break;
        case USB_TRANSFER_STATUS_ERROR:
            // if (!(trx->flags & USB_SHO)) {
            //     dev_err(libusb_get_device(trx->dev_handle),
            //             "error on endpoint {}", trx->endpoint);
            // }
            // else {
            //     //Tweaking status to complete as we received data, but all
            //     trx->status = LIBUSB_TRANSFER_COMPLETED;
            // }
            break;
        case USB_TRANSFER_STATUS_CANCELED: {
            if (!std::get<0>(unlink_found)) {
                //取消的不是自己，重新提交自己
                trx->status = USB_TRANSFER_STATUS_COMPLETED;
                esp_err_t err;
                {
                    std::shared_lock lock(callback_arg.handler.endpoint_cancellation_mutex);
                    if (callback_arg.transfer_type == USB_TRANSFER_TYPE_CTRL) {
                        SPDLOG_TRACE("尝试重新提交控制传输");
                        err = usb_host_transfer_submit_control(callback_arg.handler.host_client_handle, trx);
                    }
                    else {
                        SPDLOG_TRACE("尝试重新提交非控制传输");
                        err = usb_host_transfer_submit(trx);
                    }
                }
                if (err != ESP_OK) {
                    SPDLOG_ERROR("seqnum为{}的传输重新提交失败：{}", callback_arg.seqnum, esp_err_to_name(err));
                    //提交epipe
                    callback_arg.handler.session->submit_ret_submit(
                            UsbIpResponse::UsbIpRetSubmit::create_ret_submit_epipe_without_data(
                                    callback_arg.seqnum));
                }
                return;
            }
        }
        break;
        case USB_TRANSFER_STATUS_STALL:
            SPDLOG_ERROR("endpoint {} is stalled", trx->bEndpointAddress);
            break;
        case USB_TRANSFER_STATUS_NO_DEVICE: {
            callback_arg.handler.has_device = false;
            SPDLOG_INFO("device removed?");

            callback_arg.handler.session->submit_ret_submit(
                    UsbIpResponse::UsbIpRetSubmit::create_ret_submit_epipe_without_data(
                            callback_arg.seqnum));
            //清理数据
            usb_host_transfer_free(trx);
            delete &callback_arg;
            return;
        }
        break;
        default:
            SPDLOG_WARN("urb completion with unknown status {}", (int) trx->status);
            break;
    }
    SPDLOG_DEBUG("libusb传输了{}个字节", trx->actual_num_bytes);

    std::vector<UsbIpIsoPacketDescriptor> iso_packet_descriptors{};

    if (!std::get<0>(unlink_found)) {
        //发送ret_submit
        data_type received_data;
        if (!callback_arg.is_out) {
            if (callback_arg.transfer_type == USB_TRANSFER_TYPE_CTRL) {
                received_data = {
                        trx->data_buffer + USB_SETUP_PACKET_SIZE,
                        trx->data_buffer + trx->actual_num_bytes
                };
                assert(received_data.size()==trx->actual_num_bytes - USB_SETUP_PACKET_SIZE);
            }
            else if (callback_arg.transfer_type == USB_TRANSFER_TYPE_ISOCHRONOUS) {
                iso_packet_descriptors.resize(trx->num_isoc_packets);
                size_t iso_actual_length = 0;
                for (int i = 0; i < trx->num_isoc_packets; i++) {
                    auto &iso_packet = trx->isoc_packet_desc[i];
                    iso_actual_length += iso_packet.actual_num_bytes;
                }
                received_data.resize(iso_actual_length, 0);
                size_t received_data_offset = 0;
                size_t trx_buffer_offset = 0;
                for (int i = 0; i < trx->num_isoc_packets; i++) {
                    auto &iso_packet = trx->isoc_packet_desc[i];
                    std::memcpy(received_data.data() + received_data_offset, trx->data_buffer + trx_buffer_offset,
                                iso_packet.actual_num_bytes);
                    iso_packet_descriptors[i].offset = received_data_offset;
                    iso_packet_descriptors[i].length = iso_packet.actual_num_bytes;
                    iso_packet_descriptors[i].actual_length = iso_packet.actual_num_bytes;
                    iso_packet_descriptors[i].status = trxstat2error(iso_packet.status);

                    received_data_offset += iso_packet.actual_num_bytes;
                    trx_buffer_offset += iso_packet.num_bytes;
                }
            }
            else {
                received_data = {
                        trx->data_buffer,
                        trx->data_buffer + trx->actual_num_bytes
                };
                assert(received_data.size()==trx->actual_num_bytes);
            }
        }


        callback_arg.handler.session->submit_ret_submit(
                UsbIpResponse::UsbIpRetSubmit::create_ret_submit(
                        callback_arg.seqnum,
                        trxstat2error(trx->status),
                        0,
                        trx->num_isoc_packets,
                        std::move(received_data),
                        iso_packet_descriptors
                        )
                );
    }
    else {
        auto cmd_unlink_seqnum = std::get<1>(unlink_found);

        //发送ret_unlink
        callback_arg.handler.session->submit_ret_unlink_and_then_remove_seqnum_unlink(
                UsbIpResponse::UsbIpRetUnlink::create_ret_unlink(
                        cmd_unlink_seqnum,
                        trxstat2error(trx->status)
                        ),
                callback_arg.seqnum
                );
    }

    //清理数据
    usb_host_transfer_free(trx);
    delete &callback_arg;
}
