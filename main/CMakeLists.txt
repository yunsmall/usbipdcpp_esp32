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
# 想看到 spdlog 的 trace 日志就换用这行重新编译：trace/debug 的调用点被
# SPDLOG_ACTIVE_LEVEL 编译掉，运行时的 set_level 改不回来
#target_compile_definitions(${COMPONENT_LIB} PRIVATE SPDLOG_ACTIVE_LEVEL=SPDLOG_LEVEL_TRACE)
# 只作用于 main 自己的编译单元（target 级写法，与上面的 target_compile_options
# 一致）：SPDLOG_ACTIVE_LEVEL 决定 trace/debug 调用是否被编译掉，NDEBUG 关 assert
target_compile_definitions(${COMPONENT_LIB} PRIVATE
        SPDLOG_ACTIVE_LEVEL=SPDLOG_LEVEL_INFO
        NDEBUG
)
