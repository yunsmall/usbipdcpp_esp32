#pragma once

#include <usb/usb_host.h>

#include <string>
#include <format>
#include <vector>

#include <spdlog/spdlog.h>
#include <usbipdcpp/constant.h>


namespace usbipdcpp
{
    // 与 libusb 后端的 get_device_busid 一致，用物理端口拓扑
    // （{bus}-{port1}.{port2}...）而非 usb_host 动态地址：同一端口重插后
    // busid 不变（动态地址每次枚举递增，重插即变），客户端（usbip 工具）
    // 按 busid 匹配设备，稳定且可辨识。
    // 只能在设备活着时调用（bind 路径）：递归遍历 parent 链时，链上的 hub
    // 设备同样被本 client open（NEW_DEV 事件同样投递），open_count ≥ 1，
    // usbh 的 device_t 必然存活；usb_host_device_info 直接透传 usbh_dev_get_info
    // （usb_host.c）不要求 client 已 open，parent.dev_hdl 可直接递归查询。
    // 设备移除（DEV_GONE）后禁止调用本函数重建 busid：父设备 device_t 此时
    // 可能已释放，递归访问是 UAF。移除路径按 native_handle 直接匹配缓存的
    // busid 字符串（UsbDevice.busid），见 remove_gone_device
    inline std::string esp32_get_device_busid(usb_device_handle_t dev)
    {
        // 从设备向上收集端口号（叶→根）。root 直连设备 parent.dev_hdl == NULL
        // （hub.c 的 dev_tree_node_new(NULL, 0, ...)），parent.port_num 即 root
        // 端口号 1（HUB_ROOT_PORT_NUM，HCD 仅一个端口），得到 "1-1"
        std::vector<std::uint8_t> ports;
        usb_device_handle_t cur = dev;
        while (cur != nullptr) {
            // 深度上限防御（libusb 同样限制 8 层）：拓扑异常时终止递归
            if (ports.size() >= 8) {
                SPDLOG_ERROR("设备拓扑层级超过 8 层，终止 busid 构建");
                break;
            }
            usb_device_info_t info{};
            if (usb_host_device_info(cur, &info) != ESP_OK)
                break;
            ports.push_back(info.parent.port_num);
            if (info.parent.dev_hdl == nullptr)
                break;
            cur = info.parent.dev_hdl;
        }
        // 反转成根→叶：第一级端口用 '-'，后续层级用 '.'（与 libusb / 内核
        // usbip 的 busid 格式同构，如 "1-1.2"）
        std::string busid = "1";
        for (auto it = ports.rbegin(); it != ports.rend(); ++it) {
            busid += (it == ports.rbegin() ? "-" : ".");
            busid += std::to_string(*it);
        }
        return busid;
    }

    UsbSpeed esp32_speed_to_usb_speed(int speed);
}
