#include "mock_mouse.h"


using namespace usbipdcpp;

void MockMouseInterfaceHandler::handle_interrupt_transfer(Session &session, std::uint32_t seqnum, const UsbEndpoint &ep,
                                                      std::uint32_t transfer_flags,
                                                      std::uint32_t transfer_buffer_length, const data_type &out_data,
                                                      std::error_code &ec) {
    if (ep.is_in()) {
        data_type ret(4, 0);
        {
            std::shared_lock lock(data_mutex);
            if (left_pressed) {
                ret[0] |= 0b00000001;
            }
            if (right_pressed) {
                ret[0] |= 0b00000010;
            }
            if (middle_pressed) {
                ret[0] |= 0b00000100;
            }
            if (side_pressed) {
                ret[0] |= 0b00001000;
            }
            if (extra_pressed) {
                ret[0] |= 0b00010000;
            }
            ret[1] = move_horizontal;
            ret[2] = move_vertical;
            ret[3] = wheel_vertical;
        }

        session.submit_ret_submit(
                UsbIpResponse::UsbIpRetSubmit::create_ret_submit_ok_with_no_iso(seqnum, ret)
                );
    }
    else {
        session.submit_ret_submit(
                UsbIpResponse::UsbIpRetSubmit::create_ret_submit_epipe_without_data(seqnum)
                );
    }

}

void MockMouseInterfaceHandler::request_clear_feature(std::uint16_t feature_selector, std::uint32_t *p_status) {
    SPDLOG_WARN("unhandled request_clear_feature");
    *p_status = static_cast<std::uint32_t>(UrbStatusType::StatusEPIPE);
}

void MockMouseInterfaceHandler::request_endpoint_clear_feature(std::uint16_t feature_selector, std::uint8_t ep_address,
                                                           std::uint32_t *p_status) {
    SPDLOG_WARN("unhandled request_endpoint_clear_feature");
    *p_status = static_cast<std::uint32_t>(UrbStatusType::StatusEPIPE);
}

std::uint8_t MockMouseInterfaceHandler::request_get_interface(std::uint32_t *p_status) {
    return 0;
}

void MockMouseInterfaceHandler::request_set_interface(std::uint16_t alternate_setting, std::uint32_t *p_status) {
    if (alternate_setting != 0) {
        SPDLOG_WARN("unhandled request_set_interface");
        *p_status = static_cast<std::uint32_t>(UrbStatusType::StatusEPIPE);
    }
}

std::uint16_t MockMouseInterfaceHandler::request_get_status(std::uint32_t *p_status) {
    return 0;
}

std::uint16_t MockMouseInterfaceHandler::request_endpoint_get_status(std::uint8_t ep_address, std::uint32_t *p_status) {
    return 0;
}

void MockMouseInterfaceHandler::request_set_feature(std::uint16_t feature_selector, std::uint32_t *p_status) {
    SPDLOG_WARN("unhandled request_set_feature");
    *p_status = static_cast<std::uint32_t>(UrbStatusType::StatusEPIPE);
}

void MockMouseInterfaceHandler::request_endpoint_set_feature(std::uint16_t feature_selector, std::uint8_t ep_address,
                                                         std::uint32_t *p_status) {
    SPDLOG_WARN("unhandled request_endpoint_set_feature");
    *p_status = static_cast<std::uint32_t>(UrbStatusType::StatusEPIPE);
}

std::uint16_t MockMouseInterfaceHandler::get_report_descriptor_size() {
    return report_descriptor.size();
}

data_type MockMouseInterfaceHandler::get_report_descriptor() {
    return report_descriptor;

}

void MockMouseInterfaceHandler::handle_non_hid_request_type_control_urb(Session &session, std::uint32_t seqnum,
                                                                    const UsbEndpoint &ep, std::uint32_t transfer_flags,
                                                                    std::uint32_t transfer_buffer_length,
                                                                    const SetupPacket &setup_packet,
                                                                    const data_type &out_data, std::error_code &ec) {
    session.submit_ret_submit(UsbIpResponse::UsbIpRetSubmit::create_ret_submit_epipe_no_iso(seqnum, {}));
}

data_type MockMouseInterfaceHandler::request_get_report(std::uint8_t type, std::uint8_t report_id, std::uint16_t length,
                                                    std::uint32_t *p_status) {
    auto report_type = static_cast<HIDReportType>(type);
    if (report_type == HIDReportType::Input) {
        std::shared_lock lock(data_mutex);
        data_type result;
        switch (report_id) {
            case 0: {
                vector_append_to_net(result, (std::uint8_t) left_pressed);
                break;
            }
            case 1: {
                vector_append_to_net(result, (std::uint8_t) right_pressed);
                break;
            }
            case 2: {
                vector_append_to_net(result, (std::uint8_t) middle_pressed);
                break;
            }
            case 3: {
                vector_append_to_net(result, (std::uint8_t) side_pressed);
                break;
            }
            case 4: {
                vector_append_to_net(result, (std::uint8_t) extra_pressed);
                break;
            }
            case 5: {
                vector_append_to_net(result, (std::uint8_t) wheel_vertical);
                break;
            }
            case 6: {
                vector_append_to_net(result, (std::uint8_t) wheel_vertical);
                break;
            }
            case 7: {
                vector_append_to_net(result, (std::uint8_t) move_horizontal);
                break;
            }
            case 8: {
                vector_append_to_net(result, (std::uint8_t) move_vertical);
                break;
            }
            default: {
                SPDLOG_WARN("unhandled request_get_report");
                *p_status = static_cast<std::uint32_t>(UrbStatusType::StatusEPIPE);
            }
        }
        return {};
    }
    SPDLOG_WARN("unhandled request_get_report");
    *p_status = static_cast<std::uint32_t>(UrbStatusType::StatusEPIPE);
    return {};
}

void MockMouseInterfaceHandler::request_set_report(std::uint8_t type, std::uint8_t report_id, std::uint16_t length,
                                               const data_type &data, std::uint32_t *p_status) {
    SPDLOG_WARN("unhandled request_set_report");
    *p_status = static_cast<std::uint32_t>(UrbStatusType::StatusEPIPE);
}

data_type MockMouseInterfaceHandler::request_get_idle(std::uint8_t type, std::uint8_t report_id, std::uint16_t length,
                                                  std::uint32_t *p_status) {
    std::shared_lock lock(data_mutex);
    data_type result;
    vector_append_to_net(result, (std::uint16_t) idle_speed);
    return result;
}

void MockMouseInterfaceHandler::request_set_idle(std::uint8_t speed, std::uint32_t *p_status) {
    std::lock_guard lock(data_mutex);
    idle_speed = speed;
}