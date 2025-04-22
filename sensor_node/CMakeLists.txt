cmake_minimum_required(VERSION 3.13)

# Set PICO_SDK_PATH in environment
set(PICO_SDK_PATH "${CMAKE_CURRENT_SOURCE_DIR}/../pico-sdk")
include(${PICO_SDK_PATH}/pico_sdk_init.cmake)

project(sensor_node C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

add_executable(sensor_node
    main.c
    tsl2591.c
    mcp9700.c
    stemma_soil.c
    uart_protocol.c
)

# Enable USB output, disable UART output
pico_enable_stdio_usb(sensor_node 1)
pico_enable_stdio_uart(sensor_node 0)

# Pull in common dependencies
target_link_libraries(sensor_node
    pico_stdlib
    hardware_i2c
    hardware_adc
    hardware_uart
)

# Create map/bin/hex/uf2 output files
pico_add_extra_outputs(sensor_node)