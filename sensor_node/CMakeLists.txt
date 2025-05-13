# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.1.1)
set(toolchainVersion 14_2_Rel1)
set(picotoolVersion 2.1.1)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
set(PICO_BOARD pico CACHE STRING "Board type")

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

# Enable UART output for debug probe, disable USB output
pico_enable_stdio_uart(sensor_node 1)
pico_enable_stdio_usb(sensor_node 0)

# Pull in common dependencies
target_link_libraries(sensor_node
    pico_stdlib
    hardware_i2c
    hardware_adc
    hardware_uart
)

# Create map/bin/hex/uf2 output files
pico_add_extra_outputs(sensor_node)

# Add BME680 source file
target_sources(sensor_node PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/bme680.c
)
