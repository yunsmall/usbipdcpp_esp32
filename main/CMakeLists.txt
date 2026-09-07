# web/index.html 以二进制段嵌入固件（IDF 标准做法），HttpConfigApi.cpp 里用
# _binary_index_html_start/_end 符号取内容
idf_component_register(SRCS "esp32_usbipdcpp.cpp"
                            "WifiConfigManager.cpp"
                            "WifiConnection.cpp"
                            "ConfigConsole.cpp"
                            "HttpConfigApi.cpp"
                    INCLUDE_DIRS "."
                    PRIV_REQUIRES asio spdlog nvs_flash esp_wifi pthread usb usbipdcpp
                                 console esp_http_server esp_driver_uart
                    EMBED_FILES "web/index.html")

target_compile_options(${COMPONENT_LIB} PRIVATE -Wno-missing-field-initializers)
#target_compile_definitions(main PRIVATE SPDLOG_ACTIVE_LEVEL=SPDLOG_LEVEL_TRACE)
add_compile_definitions(
#        SPDLOG_ACTIVE_LEVEL=SPDLOG_LEVEL_TRACE
        SPDLOG_ACTIVE_LEVEL=SPDLOG_LEVEL_INFO
        NDEBUG
)
