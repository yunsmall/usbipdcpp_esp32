#pragma once

#include <usb/usb_host.h>

#include <string>
#include <format>

#include <spdlog/spdlog.h>
#include <constant.h>


namespace usbipdcpp
{
    inline std::string esp32_get_device_busid(std::uint8_t address)
    {
        return std::format("1-{}", address);
    }

    UsbSpeed esp32_speed_to_usb_speed(int speed);
}
