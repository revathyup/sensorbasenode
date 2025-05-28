# Copyright (c) 2023
# SPDX-License-Identifier: Apache-2.0

zephyr_library()
zephyr_library_sources(sensor_node.c)
zephyr_library_include_directories(.)
zephyr_library_compile_definitions(CONFIG_SENSOR_INIT_PRIORITY=70)
