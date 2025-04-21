cmake_minimum_required(VERSION 3.13)

# This is a simplified CMakeLists.txt for demonstration on Replit
# In a real environment, you would need the full Pico SDK

# Mock SDK initialization for demonstration
macro(pico_sdk_init)
    message(STATUS "Mock Pico SDK initialization")
endmacro()

# Define project
project(sensor_node C CXX)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# Initialize mock SDK
pico_sdk_init()

# Add mock executable for demonstration
add_executable(sensor_node
    main.c
    sensors.c
    uart_protocol.c
)

# These would normally link against Pico SDK libraries
# Instead, we're just declaring dependencies for documentation
target_compile_definitions(sensor_node PRIVATE
    PICO_SDK_MOCK=1
    MOCK_HARDWARE_I2C=1
    MOCK_HARDWARE_ADC=1
    MOCK_HARDWARE_UART=1
    MOCK_HARDWARE_GPIO=1
    MOCK_HARDWARE_TIMER=1
)

# Include directories
target_include_directories(sensor_node PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}
    ${CMAKE_CURRENT_LIST_DIR}/..
)

# Add a message to explain this is a mock build
add_custom_command(TARGET sensor_node POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E echo 
    "NOTE: This is a mock build for Replit demonstration. In a real environment, you would need the full Pico SDK."
)
