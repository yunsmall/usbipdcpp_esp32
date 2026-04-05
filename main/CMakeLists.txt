idf_component_register(SRCS "esp32_usbipdcpp.cpp"
                    INCLUDE_DIRS "." include
                    PRIV_REQUIRES asio spdlog nvs_flash esp_wifi pthread usb usbipdcpp)
#target_compile_definitions(main PRIVATE SPDLOG_ACTIVE_LEVEL=SPDLOG_LEVEL_TRACE)
add_compile_definitions(
#        SPDLOG_ACTIVE_LEVEL=SPDLOG_LEVEL_TRACE
        SPDLOG_ACTIVE_LEVEL=SPDLOG_LEVEL_INFO
        NDEBUG
)