cmake_minimum_required(VERSION 3.13)

# Include Pico SDK
include(pico_sdk_import.cmake)

project(sensor_node C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

add_executable(sensor_node
    sensor_node.c
    tsl2591.c
    stemma_soil.c
    mcp9700.c
)

# Enable USB output
pico_enable_stdio_usb(sensor_node 1)
pico_enable_stdio_uart(sensor_node 0)

target_include_directories(sensor_node PRIVATE ${CMAKE_CURRENT_LIST_DIR})

# Pull in common dependencies
target_link_libraries(sensor_node
    pico_stdlib
    hardware_i2c
    hardware_adc
    hardware_uart
    hardware_gpio
)

# Create map/bin/hex/uf2 output files
pico_add_extra_outputs(sensor_node)
