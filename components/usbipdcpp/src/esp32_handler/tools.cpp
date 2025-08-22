#include "esp32_handler/tools.h"

#include <spdlog/spdlog.h>

using namespace usbipdcpp;

UsbSpeed usbipdcpp::esp32_speed_to_usb_speed(int speed) {
    switch (speed) {
        case USB_SPEED_LOW:
            return UsbSpeed::Low;
        case USB_SPEED_FULL:
            return UsbSpeed::Full;
        case USB_SPEED_HIGH:
            return UsbSpeed::High;
        default:
            SPDLOG_DEBUG("unknown speed enum {}", speed);
            return UsbSpeed::Unknown;
    }
}
