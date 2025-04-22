cmake_minimum_required(VERSION 3.13)

# Set PICO_SDK_PATH in environment
set(PICO_SDK_PATH "/home/runner/workspace/pico-sdk")
include(${PICO_SDK_PATH}/pico_sdk_init.cmake)

project(minimal_sensor_node C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

add_executable(minimal_sensor_node
    main.c
)

# Enable USB output
pico_enable_stdio_usb(minimal_sensor_node 1)
pico_enable_stdio_uart(minimal_sensor_node 0)

# Pull in common dependencies
target_link_libraries(minimal_sensor_node
    pico_stdlib
    hardware_adc
    hardware_gpio
)

# Create map/bin/hex/uf2 output files
pico_add_extra_outputs(minimal_sensor_node)