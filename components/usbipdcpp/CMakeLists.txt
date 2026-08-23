set(usbipdcpp_DIR ${CMAKE_CURRENT_SOURCE_DIR}/usbipdcpp)

file(GLOB USBIPDCPP_BASE_SRC_FILES ${usbipdcpp_DIR}/src/*.cpp)
file(GLOB USBIPDCPP_DEVICE_HANDLER_SRC_FILES ${usbipdcpp_DIR}/src/DeviceHandler/*.cpp)
file(GLOB USBIPDCPP_INTERFACE_HANDLER_SRC_FILES ${usbipdcpp_DIR}/src/InterfaceHandler/*.cpp)

set(USBIPDCPP_ORIGIN_SRC_FILES
        ${USBIPDCPP_BASE_SRC_FILES}
        ${USBIPDCPP_DEVICE_HANDLER_SRC_FILES}
        ${USBIPDCPP_INTERFACE_HANDLER_SRC_FILES}
)

file(GLOB USBIPDCPP_ESP32_HANDLER_SRC_FILES ${CMAKE_CURRENT_SOURCE_DIR}/src/esp32_handler/*.cpp)

set(USBIPDCPP_ALL_SRC_FILES ${USBIPDCPP_ORIGIN_SRC_FILES} ${USBIPDCPP_ESP32_HANDLER_SRC_FILES})

idf_component_register(SRCS ${USBIPDCPP_ALL_SRC_FILES}
        INCLUDE_DIRS ${usbipdcpp_DIR}/include include
        REQUIRES asio lwip sock_utils spdlog usb pthread)

math(EXPR USBIPDCPP_MAX_TRANSFER_BUFFER_SIZE "2*1024*1024")

target_compile_definitions(
        ${COMPONENT_LIB} PRIVATE
        NDEBUG
        USBIPDCPP_MAX_TRANSFER_BUFFER_SIZE=${USBIPDCPP_MAX_TRANSFER_BUFFER_SIZE}
        # ESP-IDF 的 libc 只有 std::signal 声明没有实现，链接失败，跳过
        USBIPDCPP_DISABLE_SIGNAL
)

# usbipdcpp 库文件日志级别
set_source_files_properties(${USBIPDCPP_ORIGIN_SRC_FILES}
        PROPERTIES COMPILE_DEFINITIONS SPDLOG_ACTIVE_LEVEL=SPDLOG_LEVEL_INFO)
# ESP32 handler 文件日志级别：调试时改 INFO → DEBUG/TRACE
set_source_files_properties(${USBIPDCPP_ESP32_HANDLER_SRC_FILES}
        PROPERTIES COMPILE_DEFINITIONS SPDLOG_ACTIVE_LEVEL=SPDLOG_LEVEL_INFO)

#target_compile_definitions(
#        ${COMPONENT_LIB} PUBLIC
#        USBIPDCPP_FORCE_TRACK_PACKAGE
#)

target_compile_options(${COMPONENT_LIB} PRIVATE
        -Wno-error=maybe-uninitialized
        -Wno-missing-field-initializers
)