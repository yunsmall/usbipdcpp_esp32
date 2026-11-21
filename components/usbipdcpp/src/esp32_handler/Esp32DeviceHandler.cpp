#include "esp32_handler/Esp32DeviceHandler.h"

#include <algorithm>
#include <cstring>
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

void usbipdcpp::Esp32DeviceHandler::on_new_connection(TransferResponder &responder, error_code &ec) {
    AbstDeviceHandler::on_new_connection(responder, ec);
    all_transfer_should_stop = false;
    device_removed_ = false;
    log_heap_diag(TAG);

    // 上一个会话可能停在非 0 的 altsetting（客户端 detach 不会自动切回），
    // 新会话的客户端按 alt 0 枚举/传输会错位。libusb 后端打开设备时同样会把
    // 所有接口设回 alt 0（LibusbDeviceHandler 的 open 路径）：主机侧
    // release/claim 回 alt 0，再发 SET_INTERFACE 让设备也回到 alt 0
    for (auto &intf: handle_device.interfaces) {
        if (intf.current_altsetting == 0) {
            continue;
        }
        auto err = usb_host_interface_release(host_client_handle, native_handle, intf.interface_number);
        if (err == ESP_OK) {
            err = usb_host_interface_claim(host_client_handle, native_handle, intf.interface_number, 0);
        }
        if (err != ESP_OK) {
            SPDLOG_WARN("新会话恢复接口 {} 到 alt 0 失败：{}", intf.interface_number, esp_err_to_name(err));
            continue;
        }
        const SetupPacket set_interface{
                .request_type = 0x01, // Standard | Interface | OUT
                .request = static_cast<std::uint8_t>(StandardRequest::SetInterface),
                .value = 0,
                .index = intf.interface_number,
                .length = 0,
        };
        err = sync_control_transfer(set_interface);
        if (err != ESP_OK) {
            SPDLOG_WARN("新会话设置接口 {} alt 0 的设备请求失败：{}", intf.interface_number,
                        esp_err_to_name(err));
        }
        // 与 libusb 后端一致：设备请求失败也记 alt 0（主机侧端点表已经是
        // alt 0，保持旧值会让 find_ep 和实际端点不一致；客户端后续发
        // SET_INTERFACE 会重新纠正设备端）
        intf.current_altsetting = 0;
    }

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
        responder->submit_ret_unlink(
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
        // wLength 大于客户端 URB 的 buffer 长度时必须拒绝：usb_private.c 的
        // urb_alloc 把分配大小按 cache line ALIGN_UP，data_buffer_size 比请求
        // 长度大；usbh.c 的 urb_check_args 只比较 num_bytes <= data_buffer_size，
        // 存在对齐余量大小的绕过窗口，hcd 会读写这段没有从 socket 读入的堆
        // 内存（控制 OUT 会把未初始化的堆内容发给设备）。与 libusb 后端的
        // 同名检查一致，回 EPIPE
        if (setup_packet.length > transfer_buffer_length) [[unlikely]] {
            SPDLOG_ERROR("控制传输 wLength({}) 大于 transfer_buffer_length({})，拒绝传输",
                         setup_packet.length, transfer_buffer_length);
            responder->submit_ret_submit(
                    UsbIpResponse::UsbIpRetSubmit::create_ret_submit_epipe_without_data(seqnum, 0));
            return;
        }
        auto tweak_ret = tweak_special_requests(setup_packet);
        if (tweak_ret >= 0) {
            // tweak_ret == 0：tweak 成功（CLEAR_FEATURE 已同步完成、SET_INTERFACE
            // alt=0 无需操作），回复成功
            // tweak_ret > 0：tweak 失败（esp 错误码，如 set_interface alt!=0
            // 不支持、clear_halt 失败），不能回复成功——客户端会误以为操作
            // 已生效，按 EPIPE 回复让客户端感知失败
            if (tweak_ret == 0) {
                responder->submit_ret_submit(
                        UsbIpResponse::UsbIpRetSubmit::create_ret_submit_ok_without_data(seqnum, transfer_buffer_length));
            }
            else {
                SPDLOG_ERROR("tweak 特殊控制请求失败：seqnum={}, err={}", seqnum,
                             esp_err_to_name(static_cast<esp_err_t>(tweak_ret)));
                responder->submit_ret_submit(
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
        // num_bytes 必须带 setup 前缀：hcd_dwc.c 的 _buffer_fill_ctrl 用
        // num_bytes - sizeof(usb_setup_packet_t) 填数据阶段（不是 setup.wLength），
        // usbh.c 的 transfer_check_usb_compliance 也按 num_bytes 与 8 + wLength
        // 比较。wLength 超过 transfer_buffer_length 的请求上面已经拒绝，这里
        // 恒有 num_bytes <= 8 + transfer_buffer_length
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

    // 零长度的 interrupt 传输不能提交给 ESP-IDF 的 hcd：hcd_dwc.c 的
    // _buffer_fill_intr 用 num_qtds = num_bytes / mps 计算描述符数量，0 字节
    // 得到 0 个 QTD，随后仍以 num_qtds - 1 为索引写 xfer_desc_list[-1]
    // （数组下溢，破坏相邻内存）；_buffer_exec 还会把 desc_list_len = 0
    // 传给 usb_dwc_hal_chan_activate。usbh.c 的 transfer_check_usb_compliance
    // 对非控制 IN 只检查 num_bytes % mps，0 能通过，拦不住。0 长度传输没有
    // 数据阶段，本地按成功完成应答即可（libusb 后端可以真的提交 0 长度传输，
    // 这是 hcd 的边界缺陷）
    if (ep.attributes == static_cast<std::uint8_t>(EndpointAttributes::Interrupt) && num_bytes == 0) {
        SPDLOG_DEBUG("零长度中断传输，不提交给设备，直接完成");
        responder->submit_ret_submit(
                UsbIpResponse::UsbIpRetSubmit::create_ret_submit_ok_without_data(seqnum, 0));
        return;
    }

    auto *trx = static_cast<usb_transfer_t *>(cmd.transfer.get());

    // 非控制端点的 MPS 必须非 0：usbh.c 的 transfer_check_usb_compliance
    // 对非控制 IN 用 num_bytes % mps 校验对齐，ISO 对每个包也做同样的取模，
    // hcd_dwc.c 的 _buffer_fill_intr 同样对 mps 取模，mps 为 0 会直接除零
    // panic。MPS 取自设备端点描述符（USB_EP_DESC_GET_MPS），畸形描述符会在
    // 这里被拒绝而不是崩在 hcd 内部。控制传输不在这个范围：hcd_dwc.c 的
    // 控制路径不按 MPS 取模，用的是 pipe 自己的 MPS（usbh.c 从设备描述符的
    // bMaxPacketSize0 取得）
    if (!is_control && ep.max_packet_size == 0) [[unlikely]] {
        SPDLOG_ERROR("端点 {:02x} 的 MPS 为 0，无法提交给 hcd", ep.address);
        responder->submit_ret_submit(
                UsbIpResponse::UsbIpRetSubmit::create_ret_submit_epipe_without_data(seqnum, 0));
        return;
    }

    // ESP-IDF hcd 的描述符列表是编译期固定长度，URB 长度/包数由客户端任意
    // 指定，超出时 _buffer_fill_intr / _buffer_fill_isoc 里的 assert 直接
    // panic（不是返回错误），必须在这里提前拒绝
    if (ep.attributes == static_cast<std::uint8_t>(EndpointAttributes::Interrupt)) {
        // hcd_dwc.c 的 _buffer_fill_intr：一个 QTD 对应一个 MPS 包，
        // XFER_LIST_LEN_INTR = FRAME_LIST_LEN = USB_HAL_FRAME_LIST_LEN_32 = 32，
        // assert 条件为 (zero_len_packet) ? num_qtds + 1 : num_qtds <= 32——
        // ZERO_PACK 追加的零长度 QTD 单独占一项
        const std::uint32_t mps = ep.max_packet_size;
        const std::uint64_t qtds =
                (static_cast<std::uint64_t>(num_bytes) + mps - 1) / mps;
        const bool extra_zlp =
                is_out &&
                (transfer_flags & static_cast<std::uint32_t>(TransferFlag::URB_ZERO_PACKET)) != 0 &&
                num_bytes % mps == 0;
        if (qtds + (extra_zlp ? 1u : 0u) > 32) [[unlikely]] {
            SPDLOG_ERROR("interrupt 传输 {} 字节（MPS {}）需要 {} 个 QTD，超过 hcd 上限 32，拒绝传输",
                         num_bytes, mps, qtds + (extra_zlp ? 1u : 0u));
            responder->submit_ret_submit(
                    UsbIpResponse::UsbIpRetSubmit::create_ret_submit_epipe_without_data(seqnum, 0));
            return;
        }
    }
    else if (ep.attributes == static_cast<std::uint8_t>(EndpointAttributes::Isochronous)) {
        // hcd_dwc.c 的 _buffer_fill_isoc：total_num_desc = num_isoc_packets *
        // interval，assert(total_num_desc <= XFER_LIST_LEN_ISOC -
        // XFER_LIST_ISOC_MARGIN = 64 - 3)；interval 由 bInterval 换算为
        // 1 << (bInterval - 1)（同一文件的 pipe 配置代码）。bInterval=0 在
        // IDF 里是未定义的移位，bInterval>16 也不合法，一并拒绝
        const std::uint32_t binterval = ep.interval;
        if (binterval == 0 || binterval > 16) [[unlikely]] {
            SPDLOG_ERROR("ISO 端点 {:02x} 的 bInterval={} 非法，拒绝传输", ep.address, binterval);
            responder->submit_ret_submit(
                    UsbIpResponse::UsbIpRetSubmit::create_ret_submit_epipe_without_data(seqnum, 0));
            return;
        }
        const std::uint64_t interval = std::uint64_t{1} << (binterval - 1);
        if (trx->num_isoc_packets <= 0 ||
            static_cast<std::uint64_t>(trx->num_isoc_packets) * interval > 61) [[unlikely]] {
            SPDLOG_ERROR("ISO 传输 {} 个包（interval {}）超过 hcd 描述符列表容量 61，拒绝传输",
                         trx->num_isoc_packets, interval);
            responder->submit_ret_submit(
                    UsbIpResponse::UsbIpRetSubmit::create_ret_submit_epipe_without_data(seqnum, 0));
            return;
        }
        // ISO 的 QTD 用 in_iso/out_iso 的 xfer_size:12 字段（hal/usb_dwc_ll.h
        // 的 usb_dwc_ll_dma_qtd_t），单包超过 4095 字节会被截断：硬件只按低
        // 12 位长度传输，_buffer_parse_isoc 却按描述符原始 num_bytes 计算
        // actual，会把没传的缓冲区内容当数据回给客户端。usbh 的 compliance
        // 只检查包长是 MPS 整数倍和总和，不检查单包上限
        for (int i = 0; i < trx->num_isoc_packets; i++) {
            if (static_cast<std::uint32_t>(trx->isoc_packet_desc[i].num_bytes) > 0xFFF) [[unlikely]] {
                SPDLOG_ERROR("ISO 包 {} 长度 {} 超过 hcd QTD 的 12 位长度上限 4095，拒绝传输",
                             i, trx->isoc_packet_desc[i].num_bytes);
                responder->submit_ret_submit(
                        UsbIpResponse::UsbIpRetSubmit::create_ret_submit_epipe_without_data(seqnum, 0));
                return;
            }
        }
    }
    else if (ep.attributes == static_cast<std::uint8_t>(EndpointAttributes::Bulk)) {
        // hcd_dwc.c 的 _buffer_fill_bulk 把整个 URB 填进一个 QTD，QTD 的
        // xfer_size 是 in_non_iso/out_non_iso 的 17 位字段（hal/usb_dwc_ll.h），
        // 单次 bulk URB 超过 131071 字节会被截断（131072 截成 0）：硬件按
        // 截断后的长度传，_buffer_parse_bulk 却按 num_bytes - rem_len 报
        // actual，会把没传的缓冲区内容当数据回给客户端。usbh 和 hcd 都不检查
        // 这个上限
        if (num_bytes > 0x1FFFF) [[unlikely]] {
            SPDLOG_ERROR("bulk 传输 {} 字节超过 hcd 单个 QTD 的 17 位长度上限 131071，拒绝传输",
                         num_bytes);
            responder->submit_ret_submit(
                    UsbIpResponse::UsbIpRetSubmit::create_ret_submit_epipe_without_data(seqnum, 0));
            return;
        }
    }

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
    callback_args->short_not_ok =
            (transfer_flags & static_cast<std::uint32_t>(TransferFlag::URB_SHORT_NOT_OK)) != 0 && !is_out;
    callback_args->expected_length = is_control ? setup_packet.length : transfer_buffer_length;
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
        // On bulk OUT, USB_TRANSFER_FLAG_ZERO_PACK asks the host to append a
        // zero-length packet when the payload is an exact multiple of the
        // endpoint MPS, so the device sees a clean end-of-transfer marker.
        // 但不能对 bulk OUT 无条件加：ZERO_PACK 只由客户端 URB_ZERO_PACKET
        // 决定（trx->flags 的赋值在上面统一走 get_esp32_transfer_flags）。
        // hcd_dwc.c 的 _buffer_fill_bulk 在 flag 置位且长度为 MPS 整数倍时
        // 会追加一个零长度 QTD，无条件加会让所有普通 bulk OUT 都多发 ZLP，
        // 设备看到的数据流与客户端 URB 语义不一致（libusb 后端的
        // masking_bogus_flags 同样只允许客户端请求的 flag）
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
        // 清理已分配的资源再返回：callback_args 已从池分配（或 new），
        // transfer 所有权已转移至 callback_args->transfer，不清理则池槽位
        // 与 usb_transfer_t 同时泄漏（与重复 seqnum / 提交失败路径一致）
        callback_args->transfer.reset();
        callback_args->reset();
        if (!callback_args_pool_.free(callback_args)) {
            delete callback_args;
        }
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
            responder->submit_ret_submit(
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
    LATENCY_TRACK(*responder->latency_tracker(), seqnum,
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
        responder->submit_ret_submit(
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
    const std::size_t alloc_len = USB_SETUP_PACKET_SIZE + setup_packet.length;
    auto err = usb_host_transfer_alloc(alloc_len, 0, &transfer);
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
        // 数据阶段清零：本函数只用于 CLEAR_FEATURE 这类无数据阶段的请求，
        // 规范要求 wLength=0，但 is_clear_halt_cmd 不校验 wLength。hcd_dwc.c 的
        // _buffer_fill_ctrl 按 num_bytes - 8 的长度填数据阶段发给设备，而
        // usb_private.c 的 urb_alloc 不清零数据缓冲区，畸形请求会把未初始化的
        // 堆内容当 OUT 数据发出去；按 8 + wLength 分配并清零即可覆盖
        if (setup_packet.length > 0) {
            std::memset(transfer->data_buffer + USB_SETUP_PACKET_SIZE, 0, setup_packet.length);
        }

        // 传输结果带回给同步等待方：status 非 COMPLETED 时必须返回错误，
        // 否则 SET_INTERFACE 这类调用会把"设备没执行"当成成功
        struct sync_result {
            std::binary_semaphore semaphore{0};
            usb_transfer_status_t status = USB_TRANSFER_STATUS_COMPLETED;
        } result;

        transfer->device_handle = native_handle;
        transfer->callback = [](usb_transfer_t *transfer) {
            auto *result = static_cast<sync_result *>(transfer->context);
            if (transfer->status != USB_TRANSFER_STATUS_COMPLETED) {
                SPDLOG_ERROR("sync_control_transfer transfer status is not complete, which is {}",
                             static_cast<std::int32_t>(transfer->status));
                result->status = transfer->status;
            }
            // 传输已完成，在回调里归还给 usb_host，否则每次 CLEAR_FEATURE
            // 都泄漏一个 usb_transfer_t（函数 acquire 返回后不会释放它）。
            // 回调执行时 usb_host 已把 urb 从队列摘除（usb_host.c 的
            // _handle_pending_ep / done_ctrl_xfer 处理），此处置 free 安全
            usb_host_transfer_free(transfer);
            result->semaphore.release();
        };
        transfer->context = &result;
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
        result.semaphore.acquire();
        return result.status == USB_TRANSFER_STATUS_COMPLETED ? ESP_OK : ESP_ERR_INVALID_RESPONSE;
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
    // wIndex/wValue 只有低 8 位有效（接口号 / altsetting 编号），超范围直接
    // 拒绝：截断后可能误匹配到别的接口或把非法 alt 当成 0 静默成功
    if (setup_packet.index > 0xFF || setup_packet.value > 0xFF || setup_packet.length != 0) [[unlikely]] {
        SPDLOG_ERROR("SET_INTERFACE 参数非法：wIndex={}, wValue={}, wLength={}",
                     setup_packet.index, setup_packet.value, setup_packet.length);
        return ESP_ERR_INVALID_ARG;
    }
    const auto interface = static_cast<std::uint8_t>(setup_packet.index);
    const auto alternate = static_cast<std::uint8_t>(setup_packet.value);

    SPDLOG_DEBUG("set_interface: inf {} alt {}", interface, alternate);

    UsbInterface *target = nullptr;
    for (auto &intf: handle_device.interfaces) {
        if (intf.interface_number == interface) {
            target = &intf;
            break;
        }
    }
    if (!target) {
        SPDLOG_ERROR("SET_INTERFACE 找不到接口 {}", interface);
        return ESP_ERR_NOT_FOUND;
    }
    if (alternate >= target->endpoints.size()) {
        SPDLOG_ERROR("SET_INTERFACE 接口 {} 的 alt {} 超出 bind 时缓存的端点表", interface, alternate);
        return ESP_ERR_NOT_FOUND;
    }
    const auto old_alternate = target->current_altsetting;
    if (old_alternate == alternate) {
        SPDLOG_DEBUG("set_interface 接口 {} 已处于 alt {}，无需切换", interface, alternate);
        return ESP_OK;
    }

    // ESP-IDF 的端点属于 claim 时指定的 altsetting：usb_host.c 的
    // interface_claim 按 bAlternateSetting 解析描述符并分配端点，没有运行时
    // set_altsetting 接口，切换只能 release 再 claim。release 要求该接口的
    // 端点没有在途 URB（interface_release 检查 num_urb_inflight），客户端按
    // 规范应先停掉接口流量再发 SET_INTERFACE
    esp_err_t err = usb_host_interface_release(host_client_handle, native_handle, interface);
    if (err != ESP_OK) {
        SPDLOG_ERROR("SET_INTERFACE 释放接口 {} 失败：{}", interface, esp_err_to_name(err));
        return err;
    }
    err = usb_host_interface_claim(host_client_handle, native_handle, interface, alternate);
    if (err != ESP_OK) {
        SPDLOG_ERROR("SET_INTERFACE 接口 {} 声明 alt {} 失败：{}", interface, alternate, esp_err_to_name(err));
        auto rollback = usb_host_interface_claim(host_client_handle, native_handle, interface, old_alternate);
        if (rollback != ESP_OK) {
            SPDLOG_ERROR("SET_INTERFACE 回滚接口 {} 到 alt {} 失败：{}", interface, old_alternate,
                         esp_err_to_name(rollback));
        }
        return err;
    }

    // claim 只在主机侧登记端点，不会给设备发 SET_INTERFACE（interface_claim
    // 内部没有控制传输），设备端的 altsetting 必须自己转发
    err = sync_control_transfer(setup_packet);
    if (err != ESP_OK) {
        SPDLOG_ERROR("SET_INTERFACE 接口 {} alt {} 设备请求失败：{}", interface, alternate, esp_err_to_name(err));
        // 设备仍停在旧 alt，主机侧端点表也要退回去
        usb_host_interface_release(host_client_handle, native_handle, interface);
        auto rollback = usb_host_interface_claim(host_client_handle, native_handle, interface, old_alternate);
        if (rollback != ESP_OK) {
            SPDLOG_ERROR("SET_INTERFACE 回滚接口 {} 到 alt {} 失败：{}", interface, old_alternate,
                         esp_err_to_name(rollback));
        }
        return err;
    }

    // 更新端点 MPS 表：UVC 的 alt 0 可能没有端点、alt 1 才有等时端点，
    // 旧 alt 的地址要先清掉再登记新 alt 的
    for (const auto &ep: target->current_endpoints()) {
        transfer_operator_->endpoint_mps_map_.erase(ep.address);
    }
    target->current_altsetting = alternate;
    for (const auto &ep: target->current_endpoints()) {
        transfer_operator_->endpoint_mps_map_[ep.address] = ep.max_packet_size;
    }
    SPDLOG_DEBUG("接口 {} 已切换到 alt {}，端点数量: {}", interface, alternate, target->current_endpoints().size());
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
    LATENCY_TRACK(*handler->responder->latency_tracker(), cb->seqnum,
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
    // 入队（enqueue_ret_submit/unlink）也必须在锁内：并发完成的两个回调按拿锁
    // 顺序入队，响应顺序与完成顺序一致；锁外入队时后完成者可能先拿锁先入队，
    // 客户端看到完成顺序颠倒。锁内入队只是 vector push_back，成本极低，
    // wakeup_sender 已在锁外
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
                        handler->responder->submit_ret_submit(
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
            case USB_TRANSFER_STATUS_STALL: {
                // 控制传输的 STALL = 某个请求被设备拒了。"端点 xx 被 stall" 这句本身
                // 没有信息量，把 setup packet 解出来才知道是哪个请求——排查复合设备
                // （YubiKey 这种多接口的）时这是唯一能定位的线索
                if ((trx->bEndpointAddress & 0x7F) == 0 && trx->data_buffer != nullptr) {
                    const auto *setup = reinterpret_cast<const usb_setup_packet_t *>(trx->data_buffer);
                    SPDLOG_ERROR("endpoint {:02x} is stalled: bmRequestType={:02x}, bRequest={:02x}, "
                                 "wValue={:04x}, wIndex={:04x}, wLength={}",
                                 trx->bEndpointAddress, setup->bmRequestType, setup->bRequest,
                                 setup->wValue, setup->wIndex, setup->wLength);
                }
                else {
                    // 非控制端点（批量/中断）的 STALL 是端点级的，只有地址可说
                    SPDLOG_ERROR("endpoint {:02x} is stalled", trx->bEndpointAddress);
                }
                break;
            }
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
        // ESP-IDF 的 hcd 对控制传输的 actual_num_bytes 恒包含 setup 阶段的
        // 8 字节（hcd_dwc.c 的 _buffer_parse_ctrl：actual = num_bytes - rem_len，
        // 而 num_bytes 含 setup），与 libusb 的 actual_length（不含 setup）
        // 语义不同。必须不分方向统一扣掉：控制 OUT / 无数据阶段（wLength=0）
        // 同样带这 8 字节，只扣 IN 会让 vhci 的 urb->actual_length 比真实多 8
        if (is_control) {
            actual_length = actual_length > USB_SETUP_PACKET_SIZE
                                    ? actual_length - USB_SETUP_PACKET_SIZE
                                    : 0;
        }
        if (trx->num_isoc_packets > 0 && !cb->is_out) {
            std::uint32_t iso_actual = 0;
            for (int i = 0; i < trx->num_isoc_packets; i++)
                iso_actual += trx->isoc_packet_desc[i].actual_num_bytes;
            actual_length = iso_actual;
        }
        else if (!cb->is_out && trx->num_isoc_packets == 0 &&
                 actual_length > cb->original_transfer_buffer_length) {
            // IN 传输的 num_bytes 在提交前按 MPS 向上对齐过，hcd_dwc.c 的
            // _buffer_parse_bulk 用 actual = num_bytes - rem_len，_buffer_parse_intr
            // 的早停分支也按对齐后的包数换算，设备可能返回比客户端请求更多的
            // 字节（v5.5.0 的早停公式还多算 mps - 2）。USB/IP 的 actual_length
            // 不允许超过客户端 URB 的 transfer_buffer_length（vhci 按它校验并
            // 拷贝），超出会判 URB 失败。ISO 不走这里：描述符 actual 之和必须
            // 与会话头一致
            actual_length = cb->original_transfer_buffer_length;
        }

        // URB_SHORT_NOT_OK：IN 传输实际长度小于客户端期望时按 EPIPE 上报
        // （与 libusb 后端的 LIBUSB_TRANSFER_SHORT_NOT_OK 一致）。usb_types_stack.h
        // 的 USB_TRANSFER_FLAG_* 只有 ZERO_PACK 一个（ESP-IDF v5.5），没有对应
        // 标志，只能比较长度自己实现；期望长度在 receive_urb 里按传输类型取
        // （控制为 wLength，其余为原始 buffer 长度）
        usb_transfer_status_t effective_status = trx->status;
        if (effective_status == USB_TRANSFER_STATUS_COMPLETED && cb->short_not_ok &&
            cb->transfer_type != USB_TRANSFER_TYPE_ISOCHRONOUS &&
            actual_length < cb->expected_length) {
            effective_status = USB_TRANSFER_STATUS_ERROR;
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
            LATENCY_TRACK_END_MSG(*handler->responder->latency_tracker(), cb->unlink_cmd_seqnum, "被unlink");
            handler->responder->enqueue_ret_unlink(
                    UsbIpResponse::UsbIpRetUnlink::create_ret_unlink(
                            cb->unlink_cmd_seqnum, trxstat2error(effective_status)));
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
                            cb->seqnum, trxstat2error(effective_status), actual_length,
                            0, trx->num_isoc_packets, std::move(cb->transfer));
                }
                else {
                    ret = UsbIpResponse::UsbIpRetSubmit::create_ret_submit_with_status_and_no_data(
                            cb->seqnum, trxstat2error(effective_status), actual_length);
                    cb->transfer.reset();
                }
            }
            else {
                ret = UsbIpResponse::UsbIpRetSubmit::create_ret_submit(
                        cb->seqnum, trxstat2error(effective_status),
                        actual_length, 0, trx->num_isoc_packets,
                        std::move(cb->transfer));
            }
            ret.error_count = error_count;
            SPDLOG_DEBUG("esp32传输actual_length为{}个字节", actual_length);
            LATENCY_TRACK(*handler->responder->latency_tracker(), cb->seqnum,
                          "Esp32DeviceHandler::transfer_callback submit_ret_submit");
            handler->responder->enqueue_ret_submit(std::move(ret));
        }
    }
    handler->responder->wakeup_sender();

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
