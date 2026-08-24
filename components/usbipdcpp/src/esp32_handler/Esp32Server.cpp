#include "esp32_handler/Esp32Server.h"

#include <usbipdcpp/Session.h>
#include <usb/usb_helpers.h>
#include <esp_log.h>

#include <print>
#include <iostream>

#include <esp_pthread.h>

#include "esp32_handler/Esp32DeviceHandler.h"
#include "esp32_handler/tools.h"

const char *usbipdcpp::Esp32Server::TAG = "esp32_uspipdcpp_server";

usbipdcpp::Esp32Server::Esp32Server():
    host_client_handle(nullptr) {
    server.register_session_exit_callback([this]() {
        this->on_session_exit();
    });
}

void usbipdcpp::Esp32Server::init_client() {
    const usb_host_client_config_t client_config = {
            .max_num_event_msg = 15,
            .async = {
                    .client_event_callback = client_event_callback,
                    .callback_arg = this,
            }
    };

    auto err = usb_host_client_register(&client_config, &host_client_handle);
    ESP_LOGI(TAG, "client register status: %d", err);
    ESP_ERROR_CHECK(err);
}

void usbipdcpp::Esp32Server::client_event_callback(const usb_host_client_event_msg_t *event_msg, void *arg) {
    auto this_server = static_cast<Esp32Server *>(arg);
    if (event_msg->event == USB_HOST_CLIENT_EVENT_NEW_DEV) {
        spdlog::info("A new device detect in address {}", event_msg->new_dev.address);
        usb_device_handle_t dev_handle;
        auto err = usb_host_device_open(this_server->host_client_handle, event_msg->new_dev.address, &dev_handle);
        if (err != ESP_OK) {
            SPDLOG_ERROR("Failed to open USB device in address {}: {}", event_msg->new_dev.address,
                         esp_err_to_name(err));
        }
        else {
            std::lock_guard lock(this_server->all_host_devices_mutex);
            // NEW_DEV 对每个枚举成功的设备只投递一次（usb_host 枚举完成路径，
            // usbh.c），设备地址全局递增不复用，同一 address 不会重复出现，
            // 无需重复绑定检查（防御性检查无触发场景）
            this_server->host_devices[event_msg->new_dev.address] = dev_handle;
            // 回滚绑定中途失败（如 OOM 异常）的接口与句柄：接口可能已部分
            // claim、句柄未关闭。先 release 全部接口再 close——usb_host_device_close
            // 对仍有 claim 接口的设备返回 ESP_ERR_INVALID_STATE，必须先释放
            auto rollback_bind = [&]() {
                const usb_config_desc_t *cfg = nullptr;
                if (usb_host_get_active_config_descriptor(dev_handle, &cfg) == ESP_OK && cfg) {
                    for (int intf_i = 0; intf_i < cfg->bNumInterfaces; intf_i++) {
                        int intf_offset;
                        auto intf_desc = usb_parse_interface_descriptor(cfg, intf_i, 0, &intf_offset);
                        if (!intf_desc)
                            continue;
                        usb_host_interface_release(this_server->host_client_handle, dev_handle,
                                                   intf_desc->bInterfaceNumber);
                    }
                }
                usb_host_device_close(this_server->host_client_handle, dev_handle);
            };
            esp_err_t bind_ret;
            try {
                bind_ret = this_server->bind_host_device(dev_handle);
            }
            catch (const std::exception &e) {
                // C 回调边界：异常逃逸到 usbh（C 代码）是未定义行为，必须
                // 就地消化。bind 内部只在错误路径 close 句柄，异常路径由
                // 上方 rollback 补齐
                SPDLOG_ERROR("绑定设备时异常：{}", e.what());
                rollback_bind();
                bind_ret = ESP_FAIL;
            }
            catch (...) {
                SPDLOG_ERROR("绑定设备时未知异常");
                rollback_bind();
                bind_ret = ESP_FAIL;
            }
            if (bind_ret != ESP_OK) {
                // 绑定失败（含异常回滚路径）：句柄已关闭，从 map 移除，
                // 防止设备拔除时 remove_gone_device 对已关闭句柄误操作
                this_server->host_devices.erase(event_msg->new_dev.address);
            }
        }
    }
    else {
        spdlog::info("A device with handle {} has gone", static_cast<void *>(event_msg->dev_gone.dev_hdl));
        this_server->remove_gone_device(event_msg->dev_gone.dev_hdl);

        // 无需等待 session 线程退出：usbh 在 DEV_GONE 消息进入 client 队列
        // 之前已把所有传输回调派发完毕（usbh_process 动作顺序 EPn_HALT_FLUSH
        // 先于 PROP_GONE_EVT，回调先于 device_removed_ 置位完成），session 的
        // 传输侧已无活动，其退出路径不触碰 usb 句柄。此处的 release/close 是
        // ESP-IDF 对 DEV_GONE 事件的标准清理流程

        const usb_config_desc_t *active_config_desc = nullptr;
        auto err = usb_host_get_active_config_descriptor(event_msg->dev_gone.dev_hdl, &active_config_desc);
        if (err == ESP_OK) {
            spdlog::info("尝试释放{}的所有接口", static_cast<void *>(event_msg->dev_gone.dev_hdl));
            for (int intf_i = 0; intf_i < active_config_desc->bNumInterfaces; intf_i++) {
                int intf_offset;
                auto intf_desc = usb_parse_interface_descriptor(active_config_desc, intf_i, 0, &intf_offset);
                if (!intf_desc)
                    continue;
                // release 的第三参数是 bInterfaceNumber（接口号），
                // 跳号接口用数组下标会释放错误的接口
                err = usb_host_interface_release(this_server->host_client_handle, event_msg->dev_gone.dev_hdl,
                                                 intf_desc->bInterfaceNumber);
                if (err != ESP_OK) {
                    SPDLOG_ERROR("释放设备接口{}时出错: {}", intf_desc->bInterfaceNumber, esp_err_to_name(err));
                }
            }

            err = usb_host_device_close(this_server->host_client_handle, event_msg->dev_gone.dev_hdl);
            if (err != ESP_OK) {
                SPDLOG_ERROR("Failed to close USB device handle {}: {}",
                             static_cast<void *>(event_msg->dev_gone.dev_hdl),
                             esp_err_to_name(err));
            }
            else {
                spdlog::info("成功关闭device句柄{}", static_cast<void *>(event_msg->dev_gone.dev_hdl));
            }
        }
        else {
            SPDLOG_ERROR("无法获取设备活动配置描述符：{}", esp_err_to_name(err));
        }

    }
}

esp_err_t usbipdcpp::Esp32Server::bind_host_device(usb_device_handle_t dev) {
    usb_device_info_t dev_info;
    auto err = usb_host_device_info(dev, &dev_info);
    if (err != ESP_OK) {
        spdlog::warn("无法获取设备信息，忽略这个设备：{}", esp_err_to_name(err));
        // 失败统一关闭句柄，否则设备一直被占用无法重新绑定
        usb_host_device_close(host_client_handle, dev);
        return err;
    }

    const usb_device_desc_t *device_descriptor = nullptr;
    err = usb_host_get_device_descriptor(dev, &device_descriptor);
    if (err != ESP_OK) {
        spdlog::warn("无法获取设备描述符，忽略这个设备：{}", esp_err_to_name(err));
        usb_host_device_close(host_client_handle, dev);
        return err;
    }

    const usb_config_desc_t *active_config_desc = nullptr;
    err = usb_host_get_active_config_descriptor(dev, &active_config_desc);
    if (err) {
        spdlog::warn("无法获取设备当前的配置描述符，忽略这个设备：{}", esp_err_to_name(err));
        usb_host_device_close(host_client_handle, dev);
        return err;
    }

    SPDLOG_DEBUG("该设备有{}个interface", active_config_desc->bNumInterfaces);
    std::vector<UsbInterface> interfaces;
    for (auto intf_i = 0; intf_i < active_config_desc->bNumInterfaces; intf_i++) {
        [[maybe_unused]] auto alter_setting_num = usb_parse_interface_number_of_alternate(active_config_desc, intf_i);
        SPDLOG_DEBUG("第{}个interface有{}个altsetting", intf_i, alter_setting_num);

        int intf_offset;
        auto intf_desc = usb_parse_interface_descriptor(active_config_desc, intf_i, 0, &intf_offset);
        if (!intf_desc)
            continue;
        //只使用第一个alsetting
        // claim 的第三参数是 bInterfaceNumber（接口号）而非数组下标：
        // usbh 内部按 intf_desc->bInterfaceNumber 匹配接口对象（usbh.c 的
        // interface_claim），跳号接口（如只有 0 和 2）用下标会指向不存在的接口
        err = usb_host_interface_claim(host_client_handle, dev, intf_desc->bInterfaceNumber, 0);
        if (err != ESP_OK) {
            SPDLOG_ERROR("无法声明接口{}：{}", intf_desc->bInterfaceNumber, esp_err_to_name(err));
            // 回滚之前已成功声明的接口，否则它们会被永久占用
            for (auto claimed_i = 0; claimed_i < intf_i; claimed_i++) {
                int claimed_offset;
                auto claimed_intf = usb_parse_interface_descriptor(active_config_desc, claimed_i, 0, &claimed_offset);
                if (!claimed_intf)
                    continue;
                auto rel_ret = usb_host_interface_release(host_client_handle, dev, claimed_intf->bInterfaceNumber);
                if (rel_ret != ESP_OK) {
                    SPDLOG_ERROR("回滚释放接口{}失败: {}", claimed_intf->bInterfaceNumber, esp_err_to_name(rel_ret));
                }
            }
            usb_host_device_close(host_client_handle, dev);
            return err;
        }

        std::vector<UsbEndpoint> endpoints;
        endpoints.reserve(intf_desc->bNumEndpoints);
        for (auto ep_i = 0; ep_i < intf_desc->bNumEndpoints; ep_i++) {
            int endpoint_offset = intf_offset;
            auto ep_desc = usb_parse_endpoint_descriptor_by_index(intf_desc, ep_i, active_config_desc->wTotalLength,
                                                                  &endpoint_offset);
            endpoints.emplace_back(
                    ep_desc->bEndpointAddress,
                    ep_desc->bmAttributes,
                    ep_desc->wMaxPacketSize,
                    ep_desc->bInterval
                    );
        }
        interfaces.emplace_back(
                UsbInterface{
                        .interface_class = intf_desc->bInterfaceClass,
                        .interface_subclass = intf_desc->bInterfaceSubClass,
                        .interface_protocol = intf_desc->bInterfaceProtocol,
                        // 跳号接口（如只有接口 0 和 2）时下标≠接口号，必须按
                        // 描述符的 bInterfaceNumber 填充，SET_INTERFACE 才匹配正确
                        .interface_number = intf_desc->bInterfaceNumber,
                        // endpoints 按 altsetting 分组，这里只使用第一个 altsetting（alt 0）
                        .endpoints = {std::move(endpoints)}
                }
                //直接全用libusb控制，不用走端口
                // .with_handler<LibusbInterfaceHandler>()
                );
    }

    // 生成并打印 busid：设备插入时直接输出，省去客户端先 usbip list 查。
    // bind 时设备活着，构建端口拓扑 busid 安全（见 tools.h 注释）
    auto busid = esp32_get_device_busid(dev);
    SPDLOG_INFO("新设备 busid={} VID:PID={:04x}:{:04x}", busid,
                device_descriptor->idVendor, device_descriptor->idProduct);

    {
        std::lock_guard lock(server.get_devices_mutex());
        auto &server_available_devices = server.get_available_devices();
        auto current_device = std::make_shared<UsbDevice>(UsbDevice{
                .path = std::format("/esp32/usbipdcpp/{}/{}", 1, dev_info.dev_addr),
                .busid = busid,
                .bus_num = 1,
                .dev_num = dev_info.dev_addr,
                .speed = static_cast<std::uint32_t>(esp32_speed_to_usb_speed(dev_info.speed)),
                .vendor_id = device_descriptor->idVendor,
                .product_id = device_descriptor->idProduct,
                .device_bcd = device_descriptor->bcdDevice,
                .device_class = device_descriptor->bDeviceClass,
                .device_subclass = device_descriptor->bDeviceSubClass,
                .device_protocol = device_descriptor->bDeviceProtocol,
                .configuration_value = active_config_desc->bConfigurationValue,
                .num_configurations = device_descriptor->bNumConfigurations,
                .interfaces = std::move(interfaces),
                .ep0_in = UsbEndpoint::get_ep0_in(device_descriptor->bMaxPacketSize0),
                .ep0_out = UsbEndpoint::get_ep0_out(device_descriptor->bMaxPacketSize0),
                .handler = {}
        });
        current_device->with_handler<Esp32DeviceHandler>(dev, host_client_handle);
        server_available_devices.emplace_back(std::move(current_device));
    }
    return ESP_OK;
}

void usbipdcpp::Esp32Server::unbind_host_device(usb_device_handle_t dev) {
    usb_device_info_t dev_info;
    auto err = usb_host_device_info(dev, &dev_info);
    if (err != ESP_OK) {
        spdlog::error("无法获取设备信息：{}", esp_err_to_name(err));
        return;
    }
    {
        // 锁序先 all_host_devices_mutex 再 devices_mutex，与 stop() /
        // client_event_callback / remove_gone_device 保持一致，反向获取
        // 会与它们循环等待死锁
        std::lock_guard hlock(all_host_devices_mutex);
        // 本函数会 erase 可用设备列表（写操作），必须独占锁：shared_lock 是
        // 读锁，与 remove_gone_device 等写路径并发修改容器是数据竞争
        std::unique_lock lock(server.get_devices_mutex());
        auto &server_available_devices = server.get_available_devices();
        for (auto i = server_available_devices.begin(); i != server_available_devices.end(); ++i) {
            // 按 native_handle 匹配而非重建 busid：busid 是端口拓扑字符串，
            // 设备存活时直接用对象里 bind 时缓存的副本
            if (auto esp32_handler = std::dynamic_pointer_cast<Esp32DeviceHandler>((*i)->handler);
                esp32_handler && esp32_handler->native_handle == dev) {

                const usb_config_desc_t *active_config_desc = nullptr;
                err = usb_host_get_active_config_descriptor(dev, &active_config_desc);
                if (err != ESP_OK) {
                    SPDLOG_ERROR("无法获取设备活动配置描述符：{}", esp_err_to_name(err));
                    return;
                }
                for (int intf_i = 0; intf_i < active_config_desc->bNumInterfaces; intf_i++) {
                    int intf_offset;
                    auto intf_desc = usb_parse_interface_descriptor(active_config_desc, intf_i, 0, &intf_offset);
                    if (!intf_desc)
                        continue;
                    // release 的第三参数是 bInterfaceNumber（接口号），
                    // 跳号接口用数组下标会释放错误的接口
                    err = usb_host_interface_release(host_client_handle, dev, intf_desc->bInterfaceNumber);
                    if (err != ESP_OK) {
                        SPDLOG_ERROR("释放接口{}时出错: {}", intf_desc->bInterfaceNumber, esp_err_to_name(err));
                    }
                }
                server_available_devices.erase(i);
                // 关闭句柄并从 host_devices 移除（与 bind 时插入对应）：
                // 不 close 则 usb_host 句柄引用计数不归零，设备被本 client
                // 永久占用、其他 client 无法打开；不擦除条目则残留的句柄
                // 会在后续拔出清理路径被误操作
                err = usb_host_device_close(host_client_handle, dev);
                if (err != ESP_OK) {
                    SPDLOG_ERROR("关闭设备句柄{}失败: {}", static_cast<void *>(dev), esp_err_to_name(err));
                }
                host_devices.erase(dev_info.dev_addr);
                spdlog::info("成功取消绑定");
                return;
            }
        }
        SPDLOG_WARN("可使用的设备中无目标设备");

        auto &server_using_devices = server.get_using_devices();
        for (auto &[busid, device]: server_using_devices) {
            if (auto esp32_handler = std::dynamic_pointer_cast<Esp32DeviceHandler>(device->handler);
                esp32_handler && esp32_handler->native_handle == dev) {
                SPDLOG_WARN("正在使用的设备不支持解绑: {}", busid);
                return;
            }
        }
    }
}

usbipdcpp::error_code usbipdcpp::Esp32Server::start(asio::ip::tcp::endpoint &ep) {
    // 设置线程栈，减少内存占用。加锁防止多线程并发修改全局 pthread 配置
    server.set_before_thread_create_callback([this](ThreadPurpose purpose) {
        thread_cfg_mutex.lock();
        esp_pthread_cfg_t cfg = esp_pthread_get_default_config();
        cfg.pin_to_core = 1;
        switch (purpose) {
            case ThreadPurpose::NetworkIO:
                cfg.stack_size = 4096;
                cfg.thread_name = "usbipd_nio";
                break;
            case ThreadPurpose::SessionMain:
                cfg.stack_size = 8192;
                cfg.thread_name = "usbipd_sess";
                break;
            case ThreadPurpose::SessionSender:
                cfg.stack_size = 8192;
                cfg.thread_name = "usbipd_send";
                break;
        }
        esp_pthread_set_cfg(&cfg);
    });
    // 第二参数是线程指针，nullptr 表示线程创建失败（库保证 before/after
    // 成对调用）。本回调不访问线程对象，只恢复配置并解锁，两种路径一致
    server.set_after_thread_create_callback([this](ThreadPurpose, std::thread *) {
        esp_pthread_cfg_t default_cfg = esp_pthread_get_default_config();
        esp_pthread_set_cfg(&default_cfg);
        thread_cfg_mutex.unlock();
    });

    // Server::start 不抛异常，错误通过返回值报告（便于无异常环境的嵌入式平台）
    auto ec = server.start(ep);
    if (ec) [[unlikely]] {
        // 启动失败（如端口被占）：无需创建 client 事件线程，直接返回错误码
        return ec;
    }

    {
        // RAII 持锁：std::thread 构造抛异常（资源不足）时锁自动释放，不会
        // 因未解锁阻塞后续所有线程创建（before/after 回调也用本锁互斥）
        std::lock_guard cfg_lock(thread_cfg_mutex);
        esp_pthread_cfg_t pthread_cfg = esp_pthread_get_default_config();
        pthread_cfg.pin_to_core = 1;
        pthread_cfg.thread_name = "Esp32Server client_event_thread";
        pthread_cfg.stack_size = 5120;
        esp_pthread_set_cfg(&pthread_cfg);
        try {
            client_event_thread = std::thread([this]() {
                try {
                    SPDLOG_INFO("启动一个client event handle的事件循环线程");
                    while (!should_exit_client_event_thread) {
                        auto ret = usb_host_client_handle_events(host_client_handle,pdMS_TO_TICKS(10000));
                        if (ret == ESP_OK) [[likely]]
                                continue;
                        else if (ret == ESP_ERR_TIMEOUT) {
                            // SPDLOG_WARN("usb_host_client_handle_events timeout");
                            continue;
                        }
                        else [[unlikely]] {
                            SPDLOG_ERROR("usb_host_client_handle_events发生错误：{}", esp_err_to_name(ret));
                            break;
                        }
                    }
                    SPDLOG_TRACE("退出client event事件循环");
                } catch (const std::exception &e) {
                    SPDLOG_ERROR("An unexpected exception occurs in client event handle thread: {}", e.what());
                    std::exit(1);
                }
            });
        }
        catch (const std::system_error &e) {
            // 线程创建失败（资源不足）：start 承诺不抛异常，错误通过返回值
            // 报告（与 LibusbServer::start 一致）。调用方按失败处理不再调
            // stop()，这里必须回滚已启动的 server，否则监听端口泄漏。
            // 锁由 RAII 释放；恢复全局 pthread 默认配置，避免本次设置的
            // 栈大小/核心亲和性残留影响后续线程创建
            SPDLOG_ERROR("创建 client event 线程失败：{}", e.what());
            esp_pthread_cfg_t default_cfg = esp_pthread_get_default_config();
            esp_pthread_set_cfg(&default_cfg);
            server.stop();
            return std::make_error_code(std::errc::resource_unavailable_try_again);
        }
        esp_pthread_cfg_t default_cfg = esp_pthread_get_default_config();
        esp_pthread_set_cfg(&default_cfg);
    }
    return ec;
}

void usbipdcpp::Esp32Server::stop() {
    server.stop();

    // 先停 client 事件线程再释放接口：否则设备拔出回调（client_event_thread
    // 中执行）可能与下方的 usb_host_interface_release 并发，同一接口释放
    // 两次或操作已关闭的句柄
    should_exit_client_event_thread = true;
    if (client_event_thread.joinable()) {
        // joinable 检查：start 中 std::thread 构造失败时线程不存在（直接
        // join 抛 std::system_error），或线程已提前退出，两种情况都跳过
        usb_host_client_unblock(host_client_handle);
        spdlog::info("等待client handle事件线程结束");
        client_event_thread.join();
        spdlog::info("client handle事件线程结束");
    }

    {
        // 先取 all_host_devices_mutex 再取 devices_mutex，与 client_event_callback
        // / remove_gone_device 的锁序保持一致：若反向获取，stop() 与设备拔出
        // 事件并发时会循环等待死锁
        std::lock_guard hlock(all_host_devices_mutex);
        std::unique_lock lock(server.get_devices_mutex());
        auto &server_available_devices = server.get_available_devices();
        for (auto avail_dev_i = server_available_devices.begin(); avail_dev_i != server_available_devices.end(); ++avail_dev_i) {
            if (auto esp32_device_handler = std::dynamic_pointer_cast<Esp32DeviceHandler>((*avail_dev_i)->handler)) {
                auto device = esp32_device_handler->native_handle;
                const usb_config_desc_t *active_config_desc;
                auto err = usb_host_get_active_config_descriptor(device, &active_config_desc);
                if (err != ESP_OK) {
                    SPDLOG_ERROR("无法获取活动配置描述符：{}", esp_err_to_name(err));
                    continue;
                }
                for (int intf_i = 0; intf_i < active_config_desc->bNumInterfaces; intf_i++) {
                    int intf_offset;
                    auto intf_desc = usb_parse_interface_descriptor(active_config_desc, intf_i, 0, &intf_offset);
                    if (!intf_desc)
                        continue;
                    // release 的第三参数是 bInterfaceNumber（接口号），
                    // 跳号接口用数组下标会释放错误的接口
                    err = usb_host_interface_release(host_client_handle, device, intf_desc->bInterfaceNumber);
                    if (err) {
                        SPDLOG_ERROR("释放设备接口{}时出错: {}", intf_desc->bInterfaceNumber, esp_err_to_name(err));
                    }
                }
            }
        }
        auto &server_using_devices = server.get_using_devices();
        for (auto i = server_using_devices.begin(); i != server_using_devices.end(); ++i) {
            if (auto esp32_device_handler = std::dynamic_pointer_cast<Esp32DeviceHandler>(i->second->handler)) {
                auto device = esp32_device_handler->native_handle;
                const usb_config_desc_t *active_config_desc;
                auto err = usb_host_get_active_config_descriptor(device, &active_config_desc);
                if (err != ESP_OK) {
                    SPDLOG_ERROR("无法获取活动配置描述符：{}", esp_err_to_name(err));
                    continue;
                }
                for (int intf_i = 0; intf_i < active_config_desc->bNumInterfaces; intf_i++) {
                    int intf_offset;
                    auto intf_desc = usb_parse_interface_descriptor(active_config_desc, intf_i, 0, &intf_offset);
                    if (!intf_desc)
                        continue;
                    // release 的第三参数是 bInterfaceNumber（接口号），
                    // 跳号接口用数组下标会释放错误的接口
                    err = usb_host_interface_release(host_client_handle, device, intf_desc->bInterfaceNumber);
                    if (err) {
                        SPDLOG_ERROR("释放设备接口{}时出错: {}", intf_desc->bInterfaceNumber, esp_err_to_name(err));
                    }
                }
            }
        }

        // 接口已全部释放、句柄不再被接口占用：关闭所有设备句柄并清空记录。
        // 若不关闭，usb_host 设备句柄（引用计数管理）泄漏，设备被本 client
        // 永久占用，停止后无法重新打开。句柄关闭后 UsbDevice 中保存的
        // native_handle 失效，必须同时把设备从可用/使用列表移除（UsbDevice
        // 析构），残留的无效句柄会在后续绑定/传输中出错。此处无并发：
        // server.stop() 已等待所有 session 退出（active_sessions 归零），
        // on_disconnection 已确保全部传输回调执行完毕
        for (auto &entry: host_devices) {
            auto dev = entry.second;
            auto err = usb_host_device_close(host_client_handle, dev);
            if (err != ESP_OK) {
                SPDLOG_ERROR("关闭设备句柄{}失败: {}", static_cast<void *>(dev), esp_err_to_name(err));
            }
        }
        host_devices.clear();
        server_available_devices.clear();
        server_using_devices.clear();
    }

}

usbipdcpp::Esp32Server::~Esp32Server() {
}

void usbipdcpp::Esp32Server::on_session_exit() {
    std::lock_guard lock(server.get_devices_mutex());
    auto &server_using_devices = server.get_using_devices();
    for (auto it = server_using_devices.begin(); it != server_using_devices.end();) {
        if (auto handler = it->second->handler) {
            if (handler->is_device_removed()) {
                it = server_using_devices.erase(it);
                continue;
            }
        }
        ++it;
    }
}

void usbipdcpp::Esp32Server::remove_gone_device(usb_device_handle_t dev) {
    std::lock_guard lock(all_host_devices_mutex);
    auto find_ret = std::find_if(host_devices.begin(), host_devices.end(),
                                 [&](const auto &item) {
                                     return item.second == dev;
                                 });
    if (find_ret != host_devices.end()) {
        auto address = find_ret->first;
        host_devices.erase(find_ret);
        SPDLOG_TRACE("成功从所有设备中移除拔除的设备");
        // busid 是 bind 时缓存在 UsbDevice 里的端口拓扑字符串（见 tools.h
        // esp32_get_device_busid）。设备已移除，不能按 handle 重建 busid（父
        // 设备 device_t 可能已释放，递归访问是 UAF），改为按 native_handle
        // 直接匹配对象，busid 无需在移除路径重新生成
        std::lock_guard lock2(server.get_devices_mutex());
        auto &server_available_devices = server.get_available_devices();
        for (auto i = server_available_devices.begin(); i != server_available_devices.end(); ++i) {
            if (auto esp32_handler = std::dynamic_pointer_cast<Esp32DeviceHandler>((*i)->handler);
                esp32_handler && esp32_handler->native_handle == dev) {
                // 此处可以删除设备，因为此时因其还处于可用设备，因此没有session正在处理这个设备
                server_available_devices.erase(i);
                spdlog::info("从可用设备中移除目标设备");
                return;
            }
        }
        SPDLOG_WARN("可使用的设备中无目标设备");
        auto &server_using_devices = server.get_using_devices();
        for (auto i = server_using_devices.begin(); i != server_using_devices.end(); ++i) {
            if (auto esp32_handler = std::dynamic_pointer_cast<Esp32DeviceHandler>(i->second->handler);
                esp32_handler && esp32_handler->native_handle == dev) {
                // 通过 AbstDeviceHandler 接口通知
                esp32_handler->on_device_removed();
                // 强制关闭 Session
                SPDLOG_WARN("正在使用的设备被拔出，强制关闭 Session: {}", i->first);
                esp32_handler->trigger_session_stop();
                return;
            }
        }
    }
}
