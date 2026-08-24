#include "esp32_handler/Esp32DeviceHandler.h"

#include <algorithm>
#include <esp_log.h>

#include "usbipdcpp/Session.h"
#include "usbipdcpp/protocol.h"
#include "usbipdcpp/SetupPacket.h"
#include "usbipdcpp/constant.h"
#include "usbipdcpp/Endpoint.h"
#include "usbipdcpp/utils/LatencyTracker.h"

const char *usbipdcpp::Esp32DeviceHandler::TAG = "Esp32DeviceHandler";

usbipdcpp::Esp32DeviceHandler::Esp32DeviceHandler(UsbDevice &handle_device, usb_device_handle_t native_handle,
                                                  usb_host_client_handle_t host_client_handle) :
    AbstDeviceHandler(handle_device, std::make_unique<Esp32TransferOperator>()),
    native_handle(native_handle), host_client_handle(host_client_handle),
    transfer_operator_(static_cast<Esp32TransferOperator *>(get_transfer_operator())) {
    ESP_ERROR_CHECK(usb_host_device_info(native_handle, &device_info));
}

usbipdcpp::Esp32DeviceHandler::~Esp32DeviceHandler() = default;

namespace {

void log_heap_diag(const char *tag) {
    SPDLOG_INFO("{} heap: free={}, min_free={}, dma_free={}, dma_max_block={}, psram_free={}, psram_max_block={}", tag,
                esp_get_free_heap_size(), esp_get_minimum_free_heap_size(),
                heap_caps_get_free_size(MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL),
                heap_caps_get_largest_free_block(MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL),
                heap_caps_get_free_size(MALLOC_CAP_SPIRAM),
                heap_caps_get_largest_free_block(MALLOC_CAP_SPIRAM));
}

} // anonymous namespace

void usbipdcpp::Esp32DeviceHandler::on_new_connection(Session &current_session, error_code &ec) {
    AbstDeviceHandler::on_new_connection(current_session, ec);
    all_transfer_should_stop = false;
    device_removed_ = false;
    log_heap_diag(TAG);

    // 构建端点 MPS 查找表
    transfer_operator_->endpoint_mps_map_.clear();
    for (const auto &intf: handle_device.interfaces) {
        for (const auto &ep: intf.current_endpoints()) {
            transfer_operator_->endpoint_mps_map_[ep.address] = ep.max_packet_size;
        }
    }
    // 控制端点 0 的 MPS
    if (handle_device.ep0_in.max_packet_size > 0) {
        transfer_operator_->endpoint_mps_map_[0x80] = handle_device.ep0_in.max_packet_size;
    }
    if (handle_device.ep0_out.max_packet_size > 0) {
        transfer_operator_->endpoint_mps_map_[0x00] = handle_device.ep0_out.max_packet_size;
    }
    for (const auto &[addr, mps]: transfer_operator_->endpoint_mps_map_) {
        SPDLOG_INFO("endpoint mps: addr={:02x}, mps={}", addr, mps);
    }
}

void usbipdcpp::Esp32DeviceHandler::on_disconnection(error_code &ec) {
    all_transfer_should_stop = true;

    if (device_removed_) [[unlikely]] {
        // 设备已移除：usbh 已对设备全部端点做 HALT_FLUSH，所有传输回调已
        // 入队（在 client 事件线程排队执行），无需再 cancel——设备 gone 后
        // halt/flush/clear 无意义且可能对失效句柄报错。
        // 但必须等待计数归零再返回：device_removed_ 也可能被 NO_DEVICE 状态
        // 的传输回调提前置位（该回调只是事件队列中的第一个，其余端点回调
        // 可能尚未执行），若直接返回，session 收尾会立即销毁 handler，未
        // 执行的回调访问 handler 即 UAF。等待必然结束：usbh 已 flush 全部
        // URB，回调最终都会执行并递减计数（DEV_GONE 路径下回调先于
        // DEV_GONE 消息执行完毕——usbh_process 动作顺序 EPn_HALT_FLUSH 先于
        // PROP_GONE_EVT，usb_host_client_handle_events 每轮先 _handle_pending_ep
        // 派发回调再处理事件消息队列，依据 ESP-IDF v5.5 usb 组件源码）
        SPDLOG_WARN("设备已移除，等待全部传输回调完成");
        {
            std::unique_lock lock(transfer_complete_mutex_);
            transfer_complete_cv_.wait(lock, [this]() {
                return pending_count_ == 0;
            });
        }
        AbstDeviceHandler::on_disconnection(ec);
        return;
    }

    // 取消所有端点的传输（遍历实际端点，不依赖 tracker 快照）。
    // 控制端点（ep0）无法在此单独取消：usbh_ep_get_handle 只认接口端点表，
    // 对 ep0 返回 ESP_ERR_NOT_FOUND。挂起的控制传输会在设备正常响应/STALL
    // 后回调，回调走 all_transfer_should_stop 分支收尾递减计数，不会让下方
    // 等待永久阻塞；设备已移除的场景由上面的提前返回分支覆盖。
    // 注意：ESP-IDF v5.5 无传输超时机制（usb_transfer_t.timeout_ms 注释明确
    // "currently not supported yet"，usbh/usb_host 无超时逻辑），若设备永久
    // 无响应（硬件故障/恶意设备），挂起的控制传输既不回调也无法取消，下方
    // 等待会永久阻塞——这是框架限制，此层无法解决
    for (const auto &intf: handle_device.interfaces) {
        for (const auto &ep: intf.current_endpoints()) {
            cancel_endpoint_all_transfers(ep.address);
        }
    }

    // 等待所有传输完成
    {
        std::unique_lock lock(transfer_complete_mutex_);
        transfer_complete_cv_.wait(lock, [this]() {
            return pending_count_ == 0;
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

    // 非分块传输
    if (!found) {
        std::unique_lock lock(transfers_mutex_);
        auto it = transfers_.find(unlink_seqnum);
        if (it != transfers_.end()) {
            auto *cb = it->second;
            cb->unlinked = true;
            cb->unlink_cmd_seqnum = cmd_seqnum;
            found = true;
            // 锁内先保存端点地址再解锁：解锁后 transfer_callback 可能拿锁完成
            // 传输（erase + 释放 trx + 归还 cb），再读 cb->transfer 就是 UAF
            auto ep_addr = static_cast<usb_transfer_t *>(cb->transfer.get())->bEndpointAddress;
            lock.unlock();
            cancel_endpoint_all_transfers(ep_addr);
        }
    }

    if (!found) {
        // 不在任何表中——传输已完成（回调已发 RET_SUBMIT 或 RET_UNLINK）。
        // 此时 CMD_UNLINK 已无意义，直接同步发 RET_UNLINK(0)。
        SPDLOG_DEBUG("transfer {} 已不在传输表中，立即发送 ret_unlink {}", unlink_seqnum, cmd_seqnum);
        session->submit_ret_unlink(
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
            // tweak_ret == 0：tweak 成功（CLEAR_FEATURE 已同步完成、SET_INTERFACE
            // alt=0 无需操作），回复成功
            // tweak_ret > 0：tweak 失败（esp 错误码，如 set_interface alt!=0
            // 不支持、clear_halt 失败），不能回复成功——客户端会误以为操作
            // 已生效，按 EPIPE 回复让客户端感知失败
            if (tweak_ret == 0) {
                session->submit_ret_submit(
                        UsbIpResponse::UsbIpRetSubmit::create_ret_submit_ok_without_data(seqnum, transfer_buffer_length));
            }
            else {
                SPDLOG_ERROR("tweak 特殊控制请求失败：seqnum={}, err={}", seqnum,
                             esp_err_to_name(static_cast<esp_err_t>(tweak_ret)));
                session->submit_ret_submit(
                        UsbIpResponse::UsbIpRetSubmit::create_ret_submit_epipe_without_data(seqnum, 0));
            }
            return;
        }
    }
    else if (!interface.has_value()) [[unlikely]] {
        SPDLOG_ERROR("非控制传输却不存在目标接口");
        ec = make_error_code(ErrorType::INTERNAL_ERROR);
        return;
    }

    bool is_out = is_control ? setup_packet.is_out() : !ep.is_in();

    // num_bytes 和 MPS 对齐
    std::uint32_t num_bytes;
    if (is_control) {
        // 恶意 wLength 大于 transfer_buffer_length 时 num_bytes 会超过分配的
        // 缓冲区大小，但不会越界：提交路径（usb_host_transfer_submit_control
        // → usbh_dev_submit_ctrl_urb、usb_host_transfer_submit →
        // usbh_ep_enqueue_urb）入口都无条件调 urb_check_args（usbh.c:245）
        // 检查 num_bytes <= data_buffer_size，超了返回 ESP_ERR_INVALID_ARG，
        // 进不到 hcd_urb_enqueue 就不会碰数据缓冲区，走下方失败路径回 EPIPE
        num_bytes = USB_SETUP_PACKET_SIZE + setup_packet.length;
    }
    else {
        num_bytes = transfer_buffer_length;
        // ISO 跳过 MPS 对齐：usbh 的 transfer_check_usb_compliance 要求 ISO
        // 的 num_bytes 精确等于各包 num_bytes 之和（依据 ESP-IDF v5.5
        // usbh.c），对齐会使提交失败（ESP_ERR_INVALID_ARG）
        if (!is_out && ep.max_packet_size > 0
            && ep.attributes != static_cast<std::uint8_t>(EndpointAttributes::Isochronous)) {
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
    // 池对象复用时重置旧值，防止回调误用上一个 session 的状态
    callback_args->unlinked = false;
    callback_args->unlink_cmd_seqnum = 0;

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

    // 注册到传输表。拒绝重复 seqnum（参照 LibusbDeviceHandler）：transfers_
    // 以 seqnum 为 key，重复会让两个在途传输共用一个 entry，erase/取消
    // 语义错乱（RET_SUBMIT 与 RET_UNLINK 可能乱序），旧条目还会泄漏。
    // 协议要求 seqnum 单调递增（内核 vhci 原子递增），重复属客户端违规。
    // 本次传输不提交给设备，按 EPIPE 回复
    {
        std::unique_lock lock(transfers_mutex_);
        if (transfers_.contains(seqnum)) [[unlikely]] {
            lock.unlock();
            callback_args->transfer.reset();
            callback_args->reset();
            if (!callback_args_pool_.free(callback_args)) {
                delete callback_args;
            }
            session->submit_ret_submit(
                    UsbIpResponse::UsbIpRetSubmit::create_ret_submit_epipe_without_data(seqnum, 0));
            return;
        }
        transfers_.emplace(seqnum, callback_args);
        pending_count_.fetch_add(1, std::memory_order_release);
    }

    SPDLOG_DEBUG("submit seqnum={} {} {}", seqnum,
                 is_control ? "ctrl" :
                 ep.attributes == static_cast<std::uint8_t>(EndpointAttributes::Bulk) ? "bulk" :
                 ep.attributes == static_cast<std::uint8_t>(EndpointAttributes::Interrupt) ? "intr" : "iso",
                 is_out ? "out" : "in");
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
        // 提交失败路径无需检查 cb->unlinked 改发 RET_UNLINK：receive_urb 与
        // handle_unlink_seqnum 同在 Session receiver 线程串行执行（Session.cpp
        // 的接收循环），CMD_UNLINK 不可能在本路径执行中途置位；unlink 先到时
        // seqnum 尚未注册进 transfers_，handle_unlink 会发 RET_UNLINK(0) 兜底
        {
            std::unique_lock lock(transfers_mutex_);
            transfers_.erase(seqnum);
            // 只递减不通知：receive_urb 与 on_disconnection 同在 receiver 线程，
            // 本路径的递减必然先于等待的谓词检查执行，无需 notify 也不会丢失
            // 唤醒（参考 LibusbDeviceHandler 的同类注释）
            pending_count_.fetch_sub(1, std::memory_order_release);
        }
        callback_args->transfer.reset();
        callback_args->reset();
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
    // usb_parse_interface_descriptor 的第二参数是接口在配置描述符中的索引
    // （合法用法），此函数只用于定位端点描述符并取消端点
    // （cancel_endpoint_all_transfers 按端点地址操作），不涉及
    // claim/release 的 bInterfaceNumber 语义
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
    // 持锁调用 halt/flush/clear 不会死锁：三者都是异步入队动作
    // （usb_host_endpoint_halt/flush/clear 内部只是 usbh_ep_command 把
    // USBH_EP_CMD_* 发给 usbh_process 处理，依据 ESP-IDF v5.5 usb_host.c），
    // 不等待传输回调执行；回调线程（client_event_thread）最多阻塞到本函数
    // 释放锁（微秒级），随后可正常拿共享锁重提交
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
            // 传输已完成，在回调里归还给 usb_host，否则每次 CLEAR_FEATURE
            // 都泄漏一个 usb_transfer_t（函数 acquire 返回后不会释放它）。
            // 回调执行时 usb_host 已把 urb 从队列摘除（usb_host.c 的
            // _handle_pending_ep / done_ctrl_xfer 处理），此处置 free 安全
            usb_host_transfer_free(transfer);
        };
        transfer->context = &semaphore;
        transfer->bEndpointAddress = setup_packet.calc_ep0_address();
        transfer->num_bytes = USB_SETUP_PACKET_SIZE + setup_packet.length;

        err = usb_host_transfer_submit_control(host_client_handle, transfer);
        if (err != ESP_OK) {
            usb_host_transfer_free(transfer);
            goto error_occurred;
        }
        // 无超时等待（semaphore.acquire）：不能简单加超时——ep0 无法单独
        // 取消（usbh_ep_get_handle 只认接口端点表，对 ep0 返回 NOT_FOUND），
        // 超时返回会留下仍在飞行的传输，其回调晚于函数返回执行时会写
        // 栈上已销毁的 semaphore（UAF）。设备拔出时 usbh 会自动取消 ep0
        // 传输并触发回调，因此这里不会永久阻塞
        semaphore.acquire();
        return ESP_OK;
    }

error_occurred:
    return err;
}

int usbipdcpp::Esp32DeviceHandler::tweak_clear_halt_cmd(const SetupPacket &setup_packet) {
    auto target_endp = setup_packet.index;
    SPDLOG_INFO("tweak_clear_halt_cmd");

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

void usbipdcpp::Esp32DeviceHandler::decrement_pending_and_notify() {
    // 回调路径（usb_host 事件线程）的递减与 on_disconnection 的等待
    // （receiver 线程）跨线程并发，必须在 transfer_complete_mutex_ 下互斥，
    // 否则存在丢唤醒窗口：等待线程检查谓词为 false 后、注册睡眠前，若
    // 另一线程此时完成递减+notify，通知会丢失，on_disconnection 永久阻塞。
    // 注意：receive_urb 提交失败路径的递减不走本函数——它与 on_disconnection
    // 同在 receiver 线程串行执行，递减必然先于等待的谓词检查（谓词直接
    // 满足，等待者不会入睡），无需 notify（参考 LibusbDeviceHandler 注释）
    {
        std::lock_guard lock(transfer_complete_mutex_);
        pending_count_.fetch_sub(1, std::memory_order_release);
    }
    // notify 在锁外是标准推荐做法（cppreference），不存在丢失唤醒窗口：
    // 谓词修改（fetch_sub）在锁内完成，等待者的 wait 以原子操作"释放锁并
    // 注册睡眠"，通知者不可能在"等待者检查谓词之后、注册睡眠之前"修改
    // 谓词——递减要么发生在检查前（等待者看到谓词为真不入睡），要么在
    // wait 已注册睡眠之后（notify 必达）
    transfer_complete_cv_.notify_one();
}

void usbipdcpp::Esp32DeviceHandler::transfer_callback(usb_transfer_t *trx) {
    auto *cb = static_cast<esp32_callback_args *>(trx->context);
    auto *handler = cb->handler;

    SPDLOG_DEBUG("callback seqnum={} status={} actual={}", cb->seqnum,
                 static_cast<int>(trx->status), trx->actual_num_bytes);
    LATENCY_TRACK(handler->session->latency_tracker, cb->seqnum,
                  "Esp32DeviceHandler::transfer_callback调用");

    // 如果断连了，直接清理并返回（不发送响应）
    if (handler->all_transfer_should_stop) [[unlikely]] {
        {
            std::unique_lock lock(handler->transfers_mutex_);
            handler->transfers_.erase(cb->seqnum);
        }
        cb->transfer.reset();
        cb->reset();
        if (!handler->callback_args_pool_.free(cb))
            delete cb;
        // 递减放最后：本分支对 handler 的所有访问已结束。若提前递减，
        // on_disconnection（receiver 线程）的等待立即返回，设备移除场景
        // 下 handler 随即被销毁，本分支后续访问 handler 即 UAF
        // （见本函数末尾的完整注释）
        handler->decrement_pending_and_notify();
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
                    // 无需重置 actual_num_bytes / isoc 包 actual_num_bytes：
                    // hcd 在取消完成时统一清零（hcd_dwc.c 的 _buffer_parse_error
                    // 与 flush 路径都对被取消的 URB 写 actual_num_bytes = 0，
                    // ISO 包同样清零），CANCELED 回调读到的必为 0；重提交
                    // 完成后由 hcd 写入新值
                    trx->status = USB_TRANSFER_STATUS_COMPLETED;
                    esp_err_t err = ESP_OK;
                    bool stopping = false;
                    {
                        std::shared_lock ep_lock(handler->endpoint_cancellation_mutex);
                        // 断连检查与重提交必须在同一临界区：on_disconnection 先置
                        // all_transfer_should_stop 再取消端点（cancel_endpoint_all_transfers
                        // 持 endpoint_cancellation_mutex 排他锁）。若此处不检查直接重提交，
                        // 且提交发生在该端点取消完成之后，重提交的传输将无人取消
                        // （断连流程不会回头），回调不再到来，pending_count_ 永不归零，
                        // on_disconnection 永久阻塞。临界区保证：cancel 拿到排他锁前
                        // 完成的提交必然被其 flush；置位后进入本临界区的提交被拦下
                        if (handler->all_transfer_should_stop) {
                            stopping = true;
                        }
                        else if (cb->transfer_type == USB_TRANSFER_TYPE_CTRL) {
                            err = usb_host_transfer_submit_control(handler->host_client_handle, trx);
                        }
                        else {
                            err = usb_host_transfer_submit(trx);
                        }
                    }
                    if (stopping) {
                        // 断连中：与函数开头的 all_transfer_should_stop 分支相同，
                        // 只清理不发送响应
                        handler->transfers_.erase(cb->seqnum);
                        lock.unlock();
                        cb->transfer.reset();
                        cb->reset();
                        if (!handler->callback_args_pool_.free(cb))
                            delete cb;
                        // 递减放最后（见本函数末尾的完整注释）
                        handler->decrement_pending_and_notify();
                        return;
                    }
                    if (err != ESP_OK) {
                        SPDLOG_ERROR("seqnum为{}的传输重新提交失败：{}", cb->seqnum, esp_err_to_name(err));
                        handler->transfers_.erase(cb->seqnum);
                        lock.unlock();
                        handler->session->submit_ret_submit(
                                UsbIpResponse::UsbIpRetSubmit::create_ret_submit_epipe_without_data(cb->seqnum, 0));
                        cb->transfer.reset();
                        cb->reset();
                        if (!handler->callback_args_pool_.free(cb))
                            delete cb;
                        // 递减放最后（见本函数末尾的完整注释）
                        handler->decrement_pending_and_notify();
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

        // 统计 ISO 传输中失败的包数（协议 RET_SUBMIT 的 error_count 字段；
        // 内核 stub 由 USB 核心统计后填充，此处统计非 COMPLETED 的包等价）
        std::uint32_t error_count = 0;
        if (cb->transfer_type == USB_TRANSFER_TYPE_ISOCHRONOUS) [[unlikely]] {
            for (int i = 0; i < trx->num_isoc_packets; i++) {
                if (trx->isoc_packet_desc[i].status != USB_TRANSFER_STATUS_COMPLETED)
                    error_count++;
            }
        }

        // 从 map 移除（与 handle_unlink_seqnum 互斥）。
        // 递减不在此处：必须放到本函数末尾（见函数末尾的完整注释）
        handler->transfers_.erase(cb->seqnum);

        if (cb->unlinked) {
            LATENCY_TRACK_END_MSG(handler->session->latency_tracker, cb->unlink_cmd_seqnum, "被unlink");
            handler->session->enqueue_ret_unlink(
                    UsbIpResponse::UsbIpRetUnlink::create_ret_unlink(
                            cb->unlink_cmd_seqnum, trxstat2error(trx->status)));
            cb->transfer.reset();
        }
        else {
            UsbIpResponse::UsbIpRetSubmit ret;
            if (cb->is_out) {
                if (cb->transfer_type == USB_TRANSFER_TYPE_ISOCHRONOUS) [[unlikely]] {
                    // ISO OUT：转移所有权给响应，由 send_transfer_data 发送描述符
                    // （OUT 方向不发送数据，见 Esp32TransferOperator::send_transfer_data）。
                    // 协议要求 ISO 不分方向都返回 number_of_packets 个描述符，且
                    // 描述符 actual_length 之和必须等于 header 的 actual_length
                    ret = UsbIpResponse::UsbIpRetSubmit::create_ret_submit(
                            cb->seqnum, trxstat2error(trx->status), actual_length,
                            0, trx->num_isoc_packets, std::move(cb->transfer));
                }
                else {
                    ret = UsbIpResponse::UsbIpRetSubmit::create_ret_submit_with_status_and_no_data(
                            cb->seqnum, trxstat2error(trx->status), actual_length);
                    cb->transfer.reset();
                }
            }
            else {
                ret = UsbIpResponse::UsbIpRetSubmit::create_ret_submit(
                        cb->seqnum, trxstat2error(trx->status),
                        actual_length, 0, trx->num_isoc_packets,
                        std::move(cb->transfer));
            }
            ret.error_count = error_count;
            SPDLOG_DEBUG("esp32传输actual_length为{}个字节", actual_length);
            LATENCY_TRACK(handler->session->latency_tracker, cb->seqnum,
                          "Esp32DeviceHandler::transfer_callback submit_ret_submit");
            handler->session->enqueue_ret_submit(std::move(ret));
        }
    }
    handler->session->wakeup_sender();

    cb->reset();
    if (!handler->callback_args_pool_.free(cb))
        delete cb;

    // 递减与唤醒必须放在本函数末尾（本函数对 handler 的最后一次访问）：
    // 一旦计数归零并 notify，on_disconnection（receiver 线程）的等待即返回，
    // 设备移除场景下 session 收尾会立刻销毁 handler（设备从 using 列表 erase）。
    // 若提前递减，等待者返回后本函数仍在访问 handler（wakeup_sender /
    // callback_args_pool_ / cv），与 handler 析构并发
    // 即 UAF。放末尾保证等待者被唤醒时本回调已执行完毕；正常断连场景设备
    // 回 available 列表、handler 存活，此顺序同样无害
    handler->decrement_pending_and_notify();
}
