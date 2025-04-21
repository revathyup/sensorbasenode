# Base Node application CMake file
cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(base_node)

target_sources(app PRIVATE
  src/main.c
  src/sensor_driver.c
)

target_include_directories(app PRIVATE src)
