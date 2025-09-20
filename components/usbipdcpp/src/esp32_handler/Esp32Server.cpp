#include "esp32_handler/Esp32Server.h"

#include <Session.h>
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
            this_server->host_devices[event_msg->new_dev.address] = dev_handle;
            this_server->bind_host_device(dev_handle);
        }
    }
    else {
        spdlog::info("A device with handle {} has gone", static_cast<void *>(event_msg->dev_gone.dev_hdl));
        this_server->remove_gone_device(event_msg->dev_gone.dev_hdl);

        const usb_config_desc_t *active_config_desc = nullptr;
        auto err = usb_host_get_active_config_descriptor(event_msg->dev_gone.dev_hdl, &active_config_desc);
        if (err == ESP_OK) {
            spdlog::info("尝试释放{}的所有接口", static_cast<void *>(event_msg->dev_gone.dev_hdl));
            for (int intf_i = 0; intf_i < active_config_desc->bNumInterfaces; intf_i++) {
                err = usb_host_interface_release(this_server->host_client_handle, event_msg->dev_gone.dev_hdl, intf_i);
                if (err != ESP_OK) {
                    SPDLOG_ERROR("释放设备接口时出错: {}", esp_err_to_name(err));
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

void usbipdcpp::Esp32Server::bind_host_device(usb_device_handle_t dev) {
    usb_device_info_t dev_info;
    auto err = usb_host_device_info(dev, &dev_info);
    if (err != ESP_OK) {
        spdlog::warn("无法获取设备信息，忽略这个设备：{}", esp_err_to_name(err));
        return;
    }

    const usb_device_desc_t *device_descriptor = nullptr;
    err = usb_host_get_device_descriptor(dev, &device_descriptor);
    if (err != ESP_OK) {
        spdlog::warn("无法获取设备描述符，忽略这个设备：{}", esp_err_to_name(err));
        return;
    }

    const usb_config_desc_t *active_config_desc = nullptr;
    err = usb_host_get_active_config_descriptor(dev, &active_config_desc);
    if (err) {
        spdlog::warn("无法获取设备当前的配置描述符，忽略这个设备：{}", esp_err_to_name(err));
        return;
    }

    SPDLOG_DEBUG("该设备有{}个interface", active_config_desc->bNumInterfaces);
    std::vector<UsbInterface> interfaces;
    for (auto intf_i = 0; intf_i < active_config_desc->bNumInterfaces; intf_i++) {
        [[maybe_unused]] auto alter_setting_num = usb_parse_interface_number_of_alternate(active_config_desc, intf_i);
        SPDLOG_DEBUG("第{}个interface有{}个altsetting", intf_i, alter_setting_num);

        int intf_offset;
        auto intf_desc = usb_parse_interface_descriptor(active_config_desc, intf_i, 0, &intf_offset);
        //只使用第一个alsetting
        err = usb_host_interface_claim(host_client_handle, dev, intf_i, 0);
        if (err != ESP_OK) {
            SPDLOG_ERROR("无法声明接口{}：{}", intf_i, esp_err_to_name(err));
            return;
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
                        intf_desc->bInterfaceClass,
                        intf_desc->bInterfaceSubClass,
                        intf_desc->bInterfaceProtocol,
                        std::move(endpoints)
                }
                //直接全用libusb控制，不用走端口
                // .with_handler<LibusbInterfaceHandler>()
                );
    }

    {
        std::lock_guard lock(devices_mutex);
        auto current_device = std::make_shared<UsbDevice>(UsbDevice{
                .path = std::format("/esp32/usbipdcpp/{}/{}", 1, dev_info.dev_addr),
                .busid = esp32_get_device_busid(dev_info.dev_addr),
                .bus_num = 1,
                .dev_num = dev_info.dev_addr,
                .speed = (std::uint32_t) esp32_speed_to_usb_speed(dev_info.speed),
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
        available_devices.emplace_back(std::move(current_device));
    }
}

void usbipdcpp::Esp32Server::unbind_host_device(usb_device_handle_t dev) {
    usb_device_info_t dev_info;
    auto err = usb_host_device_info(dev, &dev_info);
    if (err != ESP_OK) {
        spdlog::error("无法获取设备信息：{}", esp_err_to_name(err));
        return;
    }
    auto taregt_busid = esp32_get_device_busid(dev_info.dev_addr);
    {
        std::shared_lock lock(devices_mutex);
        for (auto i = available_devices.begin(); i != available_devices.end(); ++i) {
            if ((*i)->busid == taregt_busid) {

                auto esp32_device_handler = std::dynamic_pointer_cast<Esp32DeviceHandler>((*i)->handler);
                if (esp32_device_handler) {
                    const usb_config_desc_t *active_config_desc = nullptr;
                    err = usb_host_get_active_config_descriptor(dev, &active_config_desc);
                    if (err != ESP_OK) {
                        SPDLOG_ERROR("无法获取设备活动配置描述符：{}", esp_err_to_name(err));
                        return;
                    }
                    for (int intf_i = 0; intf_i < active_config_desc->bNumInterfaces; intf_i++) {
                        err = usb_host_interface_release(host_client_handle, dev, intf_i);
                        if (err != ESP_OK) {
                            SPDLOG_ERROR("释放设备接口时出错: {}", esp_err_to_name(err));
                        }
                    }
                }
                available_devices.erase(i);
                spdlog::info("成功取消绑定");
                return;
            }
        }
        SPDLOG_WARN("可使用的设备中无目标设备");

        if (using_devices.contains(taregt_busid)) {
            SPDLOG_WARN("正在使用的设备不支持解绑");
        }
    }
}

void usbipdcpp::Esp32Server::start(asio::ip::tcp::endpoint &ep) {
    Server::start(ep);
    esp_pthread_cfg_t pthread_cfg = esp_pthread_get_default_config();
    pthread_cfg.pin_to_core = 1; // 设置核心1
    pthread_cfg.thread_name = "Esp32Server client_event_thread";
    esp_pthread_set_cfg(&pthread_cfg);
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
    esp_pthread_cfg_t default_cfg = esp_pthread_get_default_config();
    esp_pthread_set_cfg(&default_cfg);
}

void usbipdcpp::Esp32Server::stop() {
    Server::stop();

    {
        std::shared_lock lock(devices_mutex);

        for (auto avail_dev_i = available_devices.begin(); avail_dev_i != available_devices.end(); ++avail_dev_i) {
            if (auto esp32_device_handler = std::dynamic_pointer_cast<Esp32DeviceHandler>((*avail_dev_i)->handler)) {
                auto device = esp32_device_handler->native_handle;
                const usb_config_desc_t *active_config_desc;
                auto err = usb_host_get_active_config_descriptor(device, &active_config_desc);
                if (err != ESP_OK) {
                    SPDLOG_ERROR("无法获取活动配置描述符：{}", esp_err_to_name(err));
                    continue;
                }
                for (int intf_i = 0; intf_i < active_config_desc->bNumInterfaces; intf_i++) {
                    err = usb_host_interface_release(host_client_handle, device, intf_i);
                    if (err) {
                        SPDLOG_ERROR("释放设备接口{}时出错: {}", intf_i, esp_err_to_name(err));
                    }
                }
            }
        }

        for (auto i = using_devices.begin(); i != using_devices.end(); ++i) {
            if (auto esp32_device_handler = std::dynamic_pointer_cast<Esp32DeviceHandler>(i->second->handler)) {
                auto device = esp32_device_handler->native_handle;
                const usb_config_desc_t *active_config_desc;
                auto err = usb_host_get_active_config_descriptor(device, &active_config_desc);
                if (err != ESP_OK) {
                    SPDLOG_ERROR("无法获取活动配置描述符：{}", esp_err_to_name(err));
                    continue;
                }
                for (int intf_i = 0; intf_i < active_config_desc->bNumInterfaces; intf_i++) {
                    err = usb_host_interface_release(host_client_handle, device, intf_i);
                    if (err) {
                        SPDLOG_ERROR("释放设备接口{}时出错: {}", intf_i, esp_err_to_name(err));
                    }
                }
            }
        }
    }

    should_exit_client_event_thread = true;
    usb_host_client_unblock(host_client_handle);
    spdlog::info("等待client handle事件线程结束");
    client_event_thread.join();
    spdlog::info("client handle事件线程结束");
}

usbipdcpp::Esp32Server::~Esp32Server() {
}

void usbipdcpp::Esp32Server::on_session_exit() {
    std::lock_guard lock(devices_mutex);
    for (auto it = using_devices.begin(); it != using_devices.end(); ++it) {
        if (auto handler = it->second->handler) {
            if (auto esp32_handler = std::dynamic_pointer_cast<Esp32DeviceHandler>(handler)) {
                if (!esp32_handler->has_device) {
                    it = using_devices.erase(it);
                }
            }
        }
    }
}

void usbipdcpp::Esp32Server::if_is_esp32_then_mark_removed(std::shared_ptr<AbstDeviceHandler> handler) {
    if (auto esp32_handler = std::dynamic_pointer_cast<Esp32DeviceHandler>(handler)) {
        esp32_handler->has_device = false;
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
        auto target_busid = esp32_get_device_busid(address);

        std::lock_guard lock2(devices_mutex);
        for (auto i = available_devices.begin(); i != available_devices.end(); ++i) {
            if ((*i)->busid == target_busid) {
                if_is_esp32_then_mark_removed((*i)->handler);
                //此处可以删除设备，因为此时因其还处于可用设备，因此没有session正在处理这个设备
                available_devices.erase(i);
                spdlog::info("从可用设备中移除目标设备");
                return;
            }
        }
        SPDLOG_WARN("可使用的设备中无目标设备");
        for (auto i = using_devices.begin(); i != using_devices.end(); ++i) {
            if (i->first == target_busid) {
                if_is_esp32_then_mark_removed(i->second->handler);
                //此处不能删除设备，因为此时session还未关闭，若删除设备会导致野指针
                //标记已移除后若再收到一个URB会返回一个err，从而自然导致session关闭
                SPDLOG_WARN("标记正在使用的设备为已移除");
                return;
            }
        }
    }
}
