cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(base_node)

# Add the drivers directory to the build
add_subdirectory(drivers) 