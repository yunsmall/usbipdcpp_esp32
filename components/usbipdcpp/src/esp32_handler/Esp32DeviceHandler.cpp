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

    // 清理分块传输状态（仅分块开启时存在，关闭时这些容器恒空）。
    // disconnect 回调路径为快速收敛，有意不擦 active_chunked_eps_、不排空
    // deferred_urbs_。断连后残留的端点忙标记和排队传输必须在此清空，否则
    // 新 session 的第一笔传输可能发现端点"忙"而被误排队（排队传输本身的
    // TransferHandle 析构时会调 free_transfer_handle 正确释放 DMA 资源）。
    if (transfer_operator_->enable_chunking) {
        {
            std::lock_guard lock(active_chunked_eps_mutex_);
            active_chunked_eps_.clear();
        }
        {
            std::lock_guard lock(deferred_urbs_mutex_);
            deferred_urbs_.clear();
        }
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
                return pending_count_ == 0 && chunked_count_ == 0;
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
    //
    // 为什么用 transfer_started 而非 pending_count < transfers.size()：
    // 旧代码一次性提交所有分块，pending_count 从 N 递减，< N 即表示有分块
    // 已完成。现在一次只飞一个分块，pending_count 最多为 1，对多分块传输
    // 恒 < transfers.size()，条件永远为 true，导致 unlink 永远不 cancel 卡
    // 住的分块。transfer_started 直接反映"是否已有分块成功完成"，语义准确。
    {
        std::unique_lock lock(transfer_operator_->chunked_transfers_mutex_);
        auto it = transfer_operator_->chunked_transfers_.find(unlink_seqnum);
        if (it != transfer_operator_->chunked_transfers_.end()) {
            auto *ct = it->second;
            // ct->cb 在 receive_urb 设置，只要 ct 还在传输表中 cb 就必然存活：
            // cb 归还对象池发生在回调完成路径的 cb->transfer.reset()（释放
            // 所有 chunk 并从 map 移除 ct）之后，而 map 移除与本次查找持同一
            // 把锁互斥。因此这里读到的必定是有效且未释放的 cb。
            // 不能用遍历 ct->transfers 找 cb 的方式：回调完成路径会无锁地把
            // 已完成的 chunk 置 null 并释放 trx，遍历与回调存在数据竞争，且
            // 全部置 null 后找不到 cb 会误发 ret_unlink(0)（传输实际还在进行
            // 或即将应答，造成双重应答）。
            auto *cb = ct->cb;
            if (cb) {
                cb->unlinked = true;
                cb->unlink_cmd_seqnum = cmd_seqnum;
                found = true;
                if (ct->transfer_started) {
                    SPDLOG_INFO("handle_unlink_seqnum seqnum={}: transfer already started, skip cancel", unlink_seqnum);
                }
                else {
                    // 锁内保存端点地址，解锁后再 cancel：cancel_endpoint_all_transfers
                    // 内 halt/flush/clear 是同步调用且获取 endpoint_cancellation_mutex
                    // 排他锁，锁内调用会长时间持有 chunked_transfers_mutex_，阻塞
                    // 分块回调的查找与重提交路径。解锁后 ct 可能已被回调释放
                    // （完成路径的 free_transfer_handle 从 map 移除并归还 ct），
                    // 只能使用保存的端点地址
                    auto ep_addr = ct->ep_address;
                    lock.unlock();
                    cancel_endpoint_all_transfers(ep_addr);
                }
            }
            else {
                // 传输已分配但尚未开始（端点忙，仍在 deferred 队列等待）：
                // 从未提交过任何 chunk，不会有回调到来。直接回取消成功并
                // 标记 unlink_pending，出队时 receive_urb 发现后丢弃释放
                ct->unlink_pending = true;
                found = true;
                session->submit_ret_unlink(
                        UsbIpResponse::UsbIpRetUnlink::create_ret_unlink(cmd_seqnum, 0));
            }
        }
    }

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

    // 分块传输
    // 同端点串行化：无论分块与否，非控制传输必须逐笔排队，防止多个传输同时
    // 占用同一 pipe 导致数据交织。做法：active_chunked_eps_ 记录哪些端点
    // 正在处理传输，新来的传输若发现端点忙则入队 deferred_urbs_，等前一笔
    // 完成后由回调通过 process_pending_urb 出队投递。
    if (transfer_operator_->enable_chunking && !is_control) {
        bool is_chunked = false;
        {
            std::lock_guard lock(transfer_operator_->chunked_transfers_mutex_);
            is_chunked = transfer_operator_->chunked_transfers_.count(seqnum) > 0;
        }
        if (is_chunked) {
            auto *ct = static_cast<ChunkedTransfer *>(cmd.transfer.get());

            // 排队期间收到 CMD_UNLINK（handle_unlink_seqnum 已回 RET_UNLINK(0)
            // 并置 unlink_pending）：传输从未提交过任何 chunk，直接丢弃。
            // return 时 cmd 析构 → transfer 析构 → free_transfer_handle 释放
            // 所有 chunk 并从 map 移除 ct
            if (ct->unlink_pending) [[unlikely]] {
                SPDLOG_INFO("CHUNKED seqnum={} ep={:02x} dropped (unlinked while queued)", seqnum, ep.address);
                // 本笔若由 process_pending_urb 投递，端点已被它接管（active
                // 标记属于本笔），必须擦除并出队下一笔，否则端点残留标记；
                // Session 正常路径下 active 未插入，erase 无操作、队列空时
                // process_pending_urb 直接返回，两种情况都安全
                {
                    std::lock_guard lock(active_chunked_eps_mutex_);
                    active_chunked_eps_.erase(ep.address);
                }
                process_pending_urb(ep.address);
                return;
            }

            // 同端点有传输正在处理（分块在飞行或已有排队传输）时，新传输
            // 入队保持 FIFO 顺序。必须把"有排队传输"也计入忙：若只查
            // active_chunked_eps_，回调擦除标记后、process_pending_urb 出队
            // 前的间隙里，新到达的传输会直接提交，插到已排队传输的前面，
            // 同一端点的请求乱序。
            // 端点所有权判断（active_chunked_eps_ 记录占用端点的传输 seqnum）：
            // - active 存在且属于本笔（process_pending_urb 弹出时已原子接管
            //   端点）：本笔就是队头，可提交，队列剩项不构成忙——否则本笔
            //   会被自己重新入队，队列永不清空且无飞行传输再触发
            //   process_pending_urb，端点永久卡死
            // - active 存在且属于其他传输：端点被占用，入队
            // - active 不存在：端点空闲；新到达的传输若队列非空仍须入队
            //   （防插队），空闲则直接提交
            {
                std::lock_guard lock(active_chunked_eps_mutex_);
                std::lock_guard dlock(deferred_urbs_mutex_);
                bool busy;
                if (active_chunked_eps_.contains(ep.address)) {
                    busy = active_chunked_eps_[ep.address] != seqnum;
                }
                else {
                    busy = !deferred_urbs_[ep.address].empty();
                }
                if (busy) {
                    deferred_urbs_[ep.address].push(DeferredUrb{
                            std::move(cmd), ep, interface
                    });
                    SPDLOG_INFO("CHUNKED seqnum={} ep={:02x} queued (endpoint busy)", seqnum, ep.address);
                    return;
                }
                active_chunked_eps_[ep.address] = seqnum;
            }

            ct->seqnum = seqnum;
            ct->is_out = is_out;
            ct->transfer_buffer_length = transfer_buffer_length;
            ct->transfer_flags = transfer_flags;
            ct->total_actual_length = 0;
            ct->worst_status = USB_TRANSFER_STATUS_COMPLETED;
            ct->ep_address = ep.address;
            ct->in_short = false;
            ct->transfer_started = false;
            chunked_count_.fetch_add(1, std::memory_order_release);

            auto *cb = callback_args_pool_.alloc();
            if (!cb) [[unlikely]] cb = new esp32_callback_args{};
            cb->handler = this;
            cb->seqnum = seqnum;
            cb->is_out = is_out;
            cb->chunked = ct;
            ct->cb = cb;
            cb->transfer = std::move(cmd.transfer);
            cb->unlinked = false;
            cb->unlink_cmd_seqnum = 0;
            cb->transfer_type = (ep.attributes == static_cast<std::uint8_t>(EndpointAttributes::Bulk))
                                    ? USB_TRANSFER_TYPE_BULK
                                    : (ep.attributes == static_cast<std::uint8_t>(EndpointAttributes::Interrupt))
                                    ? USB_TRANSFER_TYPE_INTR
                                    : USB_TRANSFER_TYPE_BULK;

            // 预置所有分块的 context。
            // 必须逐分块提交（一次只飞一个 chunk），分块间隙期间 transfers 中
            // 只有少量 entry 被提交过并置了 context，其余 entry 的 context 为
            // 空。若 handle_unlink_seqnum 此时遍历 transfers 找 cb，会因碰到
            // context==null 而找不到 cb → found=false → 误发 ret_unlink(0)，
            // 但传输实际还在进行中，后续回调再发 ret_submit 造成双重应答。
            // 预置全部 context 保证 handle_unlink 任何时候都能找到有效 cb。
            for (auto *t: ct->transfers)
                t->context = cb;

            auto submit_err = submit_first_chunk(ct, cb, ep, transfer_flags);

            // 提交失败：第一个分块未进入 pipe，传输实际未开始。必须擦除端点
            // 忙标记并出队下一笔，否则端点永久被标记为 busy，后续排队传输卡死。
            // 用返回值判断而非 pending_count：传输可能极快完成（如 IN 短包），
            // 回调已把 pending_count 减到 0，此时读它会误判为提交失败，造成
            // 响应重复与 cb 双重释放
            if (submit_err != ESP_OK) {
                {
                    std::lock_guard lock(active_chunked_eps_mutex_);
                    active_chunked_eps_.erase(ct->ep_address);
                }
                // 只递减不通知：receive_urb 与 on_disconnection 同在 receiver
                // 线程，本路径的递减必然先于等待的谓词检查执行（谓词直接满足，
                // 等待者不会入睡），无需 notify 也不会丢失唤醒
                // （参考 LibusbDeviceHandler 的同类注释）
                chunked_count_.fetch_sub(1, std::memory_order_release);
                cb->transfer.reset();
                session->submit_ret_submit(
                        UsbIpResponse::UsbIpRetSubmit::create_ret_submit_epipe_without_data(seqnum, 0));
                cb->reset();
                if (!callback_args_pool_.free(cb))
                    delete cb;
                process_pending_urb(ep.address);
            }
            return;
        }
        // 非分块传输同样要走端点忙检查：若不分块直接提交，会在分块间隙（前
        // 一个 chunk 完成、下一个 chunk 尚未提交）插入 pipe，导致设备收到的
        // 数据流中掺杂了不分块传输的数据，顺序彻底打乱。因此不分块发现端点
        // 忙也必须入队，等前面传输全部结束后再投递。
        {
            std::lock_guard lock(active_chunked_eps_mutex_);
            std::lock_guard dlock(deferred_urbs_mutex_);
            // 忙判断同上方分块分支：active 属于其他传输 → 入队；active 属于
            // 本笔（process_pending_urb 刚接管）→ 可提交；active 空闲且队列
            // 非空（本笔是新到达的）→ 入队防插队；空闲且队列空 → 提交
            bool busy;
            if (active_chunked_eps_.contains(ep.address)) {
                busy = active_chunked_eps_[ep.address] != seqnum;
            }
            else {
                busy = !deferred_urbs_[ep.address].empty();
            }
            if (busy) {
                deferred_urbs_[ep.address].push(DeferredUrb{
                        std::move(cmd), ep, interface
                });
                SPDLOG_INFO("seqnum={} ep={:02x} queued (non-chunked, endpoint busy)", seqnum, ep.address);
                return;
            }
            active_chunked_eps_[ep.address] = seqnum;
        }
    }

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
    // 池对象复用时重置旧值，防止回调误用上一个 session 的状态
    callback_args->unlinked = false;
    callback_args->unlink_cmd_seqnum = 0;
    callback_args->chunked = nullptr;

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
            // 分块开启时本分支可能已在上面插入端点忙标记，必须擦除并出队
            // 下一笔，否则端点残留标记，后续同端点传输全被误排队
            if (transfer_operator_->enable_chunking && !is_control) {
                {
                    std::lock_guard lock(active_chunked_eps_mutex_);
                    active_chunked_eps_.erase(ep.address);
                }
                process_pending_urb(ep.address);
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
        // 提交失败：传输未进入 pipe，端点实际空闲。分块开启时必须擦除忙标记
        // 并出队下一笔，否则后续同端点传输全被误排队，端点永久卡死；
        // 分块关闭时从不插入标记，不执行任何操作
        if (transfer_operator_->enable_chunking && !is_control) {
            {
                std::lock_guard lock(active_chunked_eps_mutex_);
                active_chunked_eps_.erase(ep.address);
            }
            process_pending_urb(ep.address);
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

esp_err_t usbipdcpp::Esp32DeviceHandler::submit_first_chunk(ChunkedTransfer *ct, esp32_callback_args *cb,
                                                            const UsbEndpoint &ep, std::uint32_t transfer_flags) {
    auto *trx = ct->transfers[0];
    bool is_out = ct->is_out;
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
    if (is_out && ep.attributes == static_cast<std::uint8_t>(EndpointAttributes::Bulk)
        && ct->transfers.size() > 1)
        trx->flags &= ~USB_TRANSFER_FLAG_ZERO_PACK;

    ct->current_chunk = 0;
    ct->pending_count = 1;
    esp_err_t err = usb_host_transfer_submit(trx);
    if (err != ESP_OK) [[unlikely]] {
        SPDLOG_ERROR("Chunk submit failed: seqnum={}, err={}", ct->seqnum, esp_err_to_name(err));
        // 失败时不再递减 pending_count：调用方以返回值判断提交是否成功，
        // pending_count 残留值不影响后续（ct 即将被 free_transfer_handle 释放）
        trx->status = USB_TRANSFER_STATUS_ERROR;
        trx->actual_num_bytes = 0;
        if (static_cast<int>(USB_TRANSFER_STATUS_ERROR) > static_cast<int>(ct->worst_status))
            ct->worst_status = USB_TRANSFER_STATUS_ERROR;
    }
    return err;
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

void usbipdcpp::Esp32DeviceHandler::decrement_chunked_and_notify() {
    {
        std::lock_guard lock(transfer_complete_mutex_);
        chunked_count_.fetch_sub(1, std::memory_order_release);
    }
    transfer_complete_cv_.notify_one();
}

void usbipdcpp::Esp32DeviceHandler::process_pending_urb(uint8_t ep_addr) {
    DeferredUrb d;
    {
        // 锁序与 receive_urb 的忙检查一致：active_chunked_eps_mutex_ →
        // deferred_urbs_mutex_
        std::lock_guard alock(active_chunked_eps_mutex_);
        std::lock_guard lock(deferred_urbs_mutex_);
        auto it = deferred_urbs_.find(ep_addr);
        if (it == deferred_urbs_.end() || it->second.empty())
            return;
        // 端点已被其他传输占用（回调擦除标记后、本函数执行前新到达的传输
        // 提交并接管了端点）：本笔继续排队，其完成回调会再次触发本函数。
        // 与弹出+接管在同一锁内原子完成，防止与 receive_urb 的忙检查竞态
        if (active_chunked_eps_.contains(ep_addr))
            return;
        d = std::move(it->second.front());
        it->second.pop();
        if (it->second.empty())
            deferred_urbs_.erase(it);
        // 原子接管端点：active 记录本笔的 seqnum（所有权），receive_urb 的
        // 忙检查据此识别"本笔可提交"——若不加接管，出队的 URB 会被"队列
        // 非空"（剩项）挡回自己排到队尾，队列永不清空且无飞行传输再触发
        // 本函数，端点永久卡死
        active_chunked_eps_[ep_addr] = d.cmd.header.seqnum;
    }
    error_code ec;
    receive_urb(std::move(d.cmd), d.ep, d.interface, ec);
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
            cb->transfer.reset(); // 触发 free_transfer_handle，清理所有 chunk 并从 map 移除
            cb->reset();
            if (!handler->callback_args_pool_.free(cb))
                delete cb;
            // 递减放最后：本分支对 handler 的所有访问已结束。若提前递减，
            // on_disconnection（receiver 线程）的等待立即返回，设备移除场景
            // 下 handler 随即被销毁，本分支后续访问 handler 即 UAF
            // （见 transfer_callback 函数末尾的完整注释）
            handler->decrement_chunked_and_notify();
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
            // 重提交条件（全部满足才重提交）：
            // 1. 没有 STALL/NO_DEVICE 等硬错误
            // 2. IN 未收到短包（设备已结束，再提交也没用）
            // 3. 未被 unlink（或传输已开始，unlink 不能阻止数据流动）
            // 持有 chunked_transfers_mutex_ 原子化判断，防止 handle_unlink_seqnum
            // 在判断与重提交之间设置 unlinked 并取消端点
            bool has_error = (static_cast<int>(ct->worst_status) == static_cast<int>(USB_TRANSFER_STATUS_STALL) ||
                              static_cast<int>(ct->worst_status) == static_cast<int>(USB_TRANSFER_STATUS_NO_DEVICE));
            std::lock_guard ck_lock(handler->transfer_operator_->chunked_transfers_mutex_);
            if (!has_error && !ct->in_short && (!cb->unlinked || ct->transfer_started)) {
                SPDLOG_INFO("chunk seqnum={} resubmitting: {}",
                            cb->seqnum,
                            cb->unlinked ? "unlink arrived but transfer already started" :
                            "canceled by another transfer on same endpoint");
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
                SPDLOG_INFO("chunk {}/{} seqnum {} not resubmitting: {}",
                            chunk_idx + 1, ct->transfers.size(), cb->seqnum,
                            has_error ? "STALL or device removed" :
                            ct->in_short ? "IN short packet received, device done" :
                            "unlink arrived before transfer started");
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

    // 跟踪最差状态。IN 短包触发主动取消时，剩余分块的 CANCELED 不覆盖已完成的正常状态
    if (static_cast<int>(trx->status) > static_cast<int>(ct->worst_status)
        && !(ct->in_short && trx->status == USB_TRANSFER_STATUS_CANCELED))
        ct->worst_status = trx->status;

    // 立即释放 DMA transfer：OUT 数据已发给设备，IN 数据已拷入 in_data
    // 此处置 null 不持 chunked_transfers_mutex_：唯一并发读者是
    // handle_unlink_seqnum 的锁内遍历，其读 cb 的前提是 ct 仍在传输表中，
    // 而传输进行期间 cb 必然有效；指针读写原子，即使读到旧值也只得到
    // 有效 trx（cb 仍有效）或 nullptr（跳过），无实际危害。
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
        // IN 传输：设备发短包/ZLP 表示数据已全部返回，后续已提交的分块会
        // 因设备无数据而永久 NAK，无法拿到回调。必须取消端点把剩余分块清掉，
        // 让它们以 CANCELED 状态回调，触发 pending_count 归零从而提交响应。
        if (trx->actual_num_bytes < trx->num_bytes) {
            SPDLOG_INFO("IN SHORT chunk {}/{} seqnum={}: actual={} < num_bytes={}, canceling remaining",
                        chunk_idx + 1, ct->transfers.size(), cb->seqnum,
                        trx->actual_num_bytes, trx->num_bytes);
            ct->in_short = true;
            handler->cancel_endpoint_all_transfers(ct->ep_address);
        }
    }
    else {
        // IN 非 COMPLETED（或 in_data 缺失）：不拷贝数据。取消/错误时
        // actual_num_bytes 为 0，OVERFLOW 即使有字节也不累计，避免 in_data
        // 空洞与 actual_length 虚高；in_data 缺失（正常不会发生，alloc 失败
        // 会中止传输）时数据保留在 trx 中由 send_transfer_data 读取
        if (trx->status == USB_TRANSFER_STATUS_COMPLETED)
            ct->total_actual_length += trx->actual_num_bytes;
    }

    if (--ct->pending_count > 0)
        return;

    // 一次只提交一个分块，当前分块完成后在这里提交下一个。不一次提交全部
    // 的理由：ESP32 DWC 控制器每 pipe 仅 2 个 buffer，若一次提交 N 个 IN
    // 分块，它们占满 pipe 后设备 NAK（无数据），pipe 阻塞，其他端点的
    // OUT 传输也无法被控制器调度，整个设备停止响应。
    {
        bool has_error = (static_cast<int>(ct->worst_status) == static_cast<int>(USB_TRANSFER_STATUS_STALL) ||
                          static_cast<int>(ct->worst_status) == static_cast<int>(USB_TRANSFER_STATUS_NO_DEVICE));
        int next = ct->current_chunk + 1;
        // unlinked 检查与提交必须在同一临界区：若先读 unlinked 再解锁提交，
        // handle_unlink_seqnum 可能在间隙设置 unlinked 并完成 cancel（pipe 已
        // 恢复 ACTIVE），本提交仍会进入 pipe 把数据发给设备，违背 unlink
        // 语义。锁内提交与 handle_unlink_seqnum 的锁序一致（chunked_transfers_mutex_
        // → endpoint_cancellation_mutex），无死锁；ESP32-P4 为 RISC-V RVWMO
        // 弱内存序，持锁读保证看到 unlinked 的最新值
        std::lock_guard ck_lock(handler->transfer_operator_->chunked_transfers_mutex_);
        if (!has_error && !ct->in_short && next < static_cast<int>(ct->transfers.size())
            && (!cb->unlinked || ct->transfer_started)) {
            auto *next_trx = ct->transfers[next];
            if (next_trx) {
                std::uint32_t chunk_bytes = ct->is_out ? next_trx->num_bytes : next_trx->data_buffer_size;
                next_trx->device_handle = handler->native_handle;
                next_trx->callback = chunked_transfer_callback;
                next_trx->context = cb;
                next_trx->bEndpointAddress = ct->ep_address;
                next_trx->num_bytes = chunk_bytes;
                next_trx->flags = get_esp32_transfer_flags(ct->transfer_flags);
                if (ct->is_out && next != static_cast<int>(ct->transfers.size()) - 1)
                    next_trx->flags &= ~USB_TRANSFER_FLAG_ZERO_PACK;
                ct->current_chunk = next;
                ct->pending_count = 1;
                esp_err_t err;
                {
                    std::shared_lock ep_lock(handler->endpoint_cancellation_mutex);
                    err = usb_host_transfer_submit(next_trx);
                }
                if (err == ESP_OK)
                    return;
                SPDLOG_ERROR("Chunk submit failed: seqnum={}, chunk={}/{}, err={}",
                             ct->seqnum, next + 1, ct->transfers.size(), esp_err_to_name(err));
                next_trx->status = USB_TRANSFER_STATUS_ERROR;
                next_trx->actual_num_bytes = 0;
                if (static_cast<int>(USB_TRANSFER_STATUS_ERROR) > static_cast<int>(ct->worst_status))
                    ct->worst_status = USB_TRANSFER_STATUS_ERROR;
                --ct->pending_count;
                // 提交失败无回调，继续走完成流程
            }
        }
    }

    // ========== 所有 chunk 完成 ==========
    // 先擦除端点忙标记再发送响应。若先发响应再擦除，sender 线程被唤醒后
    // host 可能立刻发来下一笔同端点传输，而标记尚未擦除，新传输被误排队。
    // 标记擦除和 process_pending_urb 分离：前者释放端点，后者出队下一笔。
    // 必须分两步——若在持 chunked_transfers_mutex_ 时调 process_pending_urb，
    // 其内部 receive_urb 可能拿 active_chunked_eps_mutex_，与锁序冲突。
    // 顺序保证：receive_urb 的忙检查把"deferred 队列非空"也算作忙，因此
    // 本端点只要还有排队传输，新到达的传输只会入队到队尾而不是直接提交，
    // 出队的传输始终先于新传输进入 pipe。
    {
        std::lock_guard lock(handler->active_chunked_eps_mutex_);
        handler->active_chunked_eps_.erase(ct->ep_address);
    }

    auto seqnum = ct->seqnum;
    auto worst = ct->worst_status;
    bool is_out = ct->is_out;

    // 计算 actual_length：成功传输的字节数
    std::uint32_t actual_length = static_cast<std::uint32_t>(ct->total_actual_length.load());

    // 在锁内检查 unlinked、入队响应——原子完成。
    // map 由 free_transfer_handle 统一擦除，不在此处擦，避免 free_transfer_handle
    // 查找不到 ct 而将 ChunkedTransfer* 误当 usb_transfer_t* 释放。
    {
        std::lock_guard lock(handler->transfer_operator_->chunked_transfers_mutex_);
        if (cb->unlinked) {
            SPDLOG_INFO("CHUNKED DONE seqnum={} {} UNLINK unlink_cmd={} worst_status={} actual_length={}",
                        seqnum, is_out ? "OUT" : "IN", cb->unlink_cmd_seqnum,
                        static_cast<int>(worst), actual_length);
            LATENCY_TRACK_END_MSG(handler->session->latency_tracker, cb->unlink_cmd_seqnum, "被unlink");
            handler->session->enqueue_ret_unlink(
                    UsbIpResponse::UsbIpRetUnlink::create_ret_unlink(
                            cb->unlink_cmd_seqnum, handler->trxstat2error(worst)));
        }
        else {
            UsbIpResponse::UsbIpRetSubmit ret;
            if (is_out) {
                ret = UsbIpResponse::UsbIpRetSubmit::create_ret_submit_with_status_and_no_data(
                        seqnum, handler->trxstat2error(worst), actual_length);
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
        // 递减不在此处：必须放到本函数末尾（见函数末尾的完整注释）
    }
    // 在锁外 reset，避免 free_transfer_handle 内拿 chunked_transfers_mutex_ 死锁。
    // 先捕获 ep_address，因为 cb->transfer.reset() → free_transfer_handle 可能
    // 归还/删除 ct（含 ep_address），之后不能再读 ct。
    auto done_ep = ct->ep_address;
    cb->transfer.reset();
    handler->session->wakeup_sender();
    // 出队同端点下一笔排队传输。必须在 wakeup_sender 之后调用，否则 sender
    // 被唤醒后发现队列空（新传输尚未入队），可能空转一轮。
    handler->process_pending_urb(done_ep);

    cb->reset();
    if (!handler->callback_args_pool_.free(cb))
        delete cb;

    // 递减必须放在本函数末尾（本函数对 handler 的最后一次访问）：
    // 一旦计数归零并 notify，on_disconnection（receiver 线程）的等待即返回，
    // 设备移除场景下 session 收尾会立刻销毁 handler。若提前递减，等待者
    // 返回后本函数仍在访问 handler（wakeup_sender / process_pending_urb /
    // callback_args_pool_），与 handler 析构并发即 UAF。放末尾保证等待者
    // 被唤醒时本回调已执行完毕（on_disconnection 的等待见其函数内注释）
    handler->decrement_chunked_and_notify();
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

    // 先保存端点地址再进锁：锁内 cb->transfer.reset() 会把 trx 归还给
    // usb_host，锁外清理端点忙标记时再读 trx 字段就是 UAF
    auto ep_addr = trx->bEndpointAddress;
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
                        // 先保存端点地址再释放 trx：cb->transfer.reset() 会把
                        // trx 归还给 usb_host，之后访问 trx 任何字段都是 UAF
                        auto ep_addr = trx->bEndpointAddress;
                        handler->session->submit_ret_submit(
                                UsbIpResponse::UsbIpRetSubmit::create_ret_submit_epipe_without_data(cb->seqnum, 0));
                        cb->transfer.reset();
                        cb->reset();
                        if (!handler->callback_args_pool_.free(cb))
                            delete cb;
                        // 重提交失败意味着传输彻底结束而非继续飞行。分块开启时
                        // 必须清理端点忙标记并出队下一笔，否则 active_chunked_eps_
                        // 中该端点永久残留，后续同端点传输全被误排队，端点卡死；
                        // 分块关闭时从不插入标记，不执行任何操作
                        if (handler->transfer_operator_->enable_chunking && (ep_addr & 0x7F) != 0) {
                            {
                                std::lock_guard alock(handler->active_chunked_eps_mutex_);
                                handler->active_chunked_eps_.erase(ep_addr);
                            }
                            handler->process_pending_urb(ep_addr);
                        }
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

    // 非控制端点：分块开启时擦除端点忙标记、出队下一笔排队传输（同端点
    // 串行化是分块机制的一部分，非分块传输完成后的接力出队不能缺，否则
    // 队列卡死）。分块关闭时 receive_urb 从不插入标记、从不入队，这里
    // 不执行任何操作，正常路径不含分块逻辑。
    // 注意 disconnect 路径（all_transfer_should_stop）有意不执行此清理——
    // 断连期间不应再投递新传输，残留标记由 on_new_connection 统一清空。
    if (handler->transfer_operator_->enable_chunking && (ep_addr & 0x7F) != 0) {
        {
            std::lock_guard lock(handler->active_chunked_eps_mutex_);
            handler->active_chunked_eps_.erase(ep_addr);
        }
        handler->process_pending_urb(ep_addr);
    }

    cb->reset();
    if (!handler->callback_args_pool_.free(cb))
        delete cb;

    // 递减与唤醒必须放在本函数末尾（本函数对 handler 的最后一次访问）：
    // 一旦计数归零并 notify，on_disconnection（receiver 线程）的等待即返回，
    // 设备移除场景下 session 收尾会立刻销毁 handler（设备从 using 列表 erase）。
    // 若提前递减，等待者返回后本函数仍在访问 handler（wakeup_sender /
    // process_pending_urb / callback_args_pool_ / cv），与 handler 析构并发
    // 即 UAF。放末尾保证等待者被唤醒时本回调已执行完毕；正常断连场景设备
    // 回 available 列表、handler 存活，此顺序同样无害
    handler->decrement_pending_and_notify();
}
