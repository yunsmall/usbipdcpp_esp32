set(usbipdcpp_DIR ${CMAKE_CURRENT_SOURCE_DIR}/usbipdcpp)

file(GLOB USBIPDCPP_BASE_SRC_FILES ${usbipdcpp_DIR}/src/*.cpp)
file(GLOB USBIPDCPP_DEVICE_HANDLER_SRC_FILES ${usbipdcpp_DIR}/src/DeviceHandler/*.cpp)
file(GLOB USBIPDCPP_INTERFACE_HANDLER_SRC_FILES ${usbipdcpp_DIR}/src/InterfaceHandler/*.cpp)

set(USBIPDCPP_ORIGIN_SRC_FILES
        ${USBIPDCPP_BASE_SRC_FILES}
        ${USBIPDCPP_DEVICE_HANDLER_SRC_FILES}
        ${USBIPDCPP_INTERFACE_HANDLER_SRC_FILES}
        ${usbipdcpp_DIR}/examples/mock_mouse/mock_mouse.cpp
)

file(GLOB USBIPDCPP_ESP32_HANDLER_SRC_FILES ${CMAKE_CURRENT_SOURCE_DIR}/src/esp32_handler/*.cpp)

set(USBIPDCPP_ALL_SRC_FILES ${USBIPDCPP_ORIGIN_SRC_FILES} ${USBIPDCPP_ESP32_HANDLER_SRC_FILES})

idf_component_register(SRCS ${USBIPDCPP_ALL_SRC_FILES}
        INCLUDE_DIRS ${usbipdcpp_DIR}/include ${usbipdcpp_DIR}/examples/mock_mouse include
        REQUIRES asio lwip sock_utils spdlog usb pthread)

target_compile_definitions(
        ${COMPONENT_LIB} PRIVATE
        #        SPDLOG_ACTIVE_LEVEL=SPDLOG_LEVEL_TRACE
        SPDLOG_ACTIVE_LEVEL=SPDLOG_LEVEL_INFO
        NDEBUG
)

#target_compile_definitions(
#        ${COMPONENT_LIB} PUBLIC
#        USBIPDCPP_FORCE_TRACK_PACKAGE
#)

target_compile_options(${COMPONENT_LIB} PRIVATE
        -Wno-error=maybe-uninitialized
)