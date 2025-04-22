cmake_minimum_required(VERSION 3.13)

# Set PICO_SDK_PATH in environment
set(PICO_SDK_PATH "${CMAKE_CURRENT_SOURCE_DIR}/../pico-sdk")
include(${PICO_SDK_PATH}/pico_sdk_init.cmake)

project(base_node C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

add_executable(base_node
    main.c
    uart_protocol.c
)

# Enable USB output, disable UART output
pico_enable_stdio_usb(base_node 1)
pico_enable_stdio_uart(base_node 0)

# Pull in common dependencies
target_link_libraries(base_node
    pico_stdlib
    hardware_uart
)

# Create map/bin/hex/uf2 output files
pico_add_extra_outputs(base_node)