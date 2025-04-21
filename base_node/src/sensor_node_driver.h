/**
 * @file sensor_node_driver.h
 * @brief Zephyr sensor driver for the sensor node
 */

#ifndef SENSOR_NODE_DRIVER_H
#define SENSOR_NODE_DRIVER_H

#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/uart.h>
#include "protocol.h"

/**
 * @brief Get the device instance for the sensor node driver
 * 
 * @return Pointer to the sensor node device or NULL if not available
 */
const struct device *sensor_node_driver_get_device(void);

/**
 * @brief Configure the sensor node
 * 
 * @param dev Device to configure
 * @param param Parameter to configure
 * @param value Value to set
 * @return 0 if successful, negative error code otherwise
 */
int sensor_node_set_config(const struct device *dev, enum config_param param, int32_t value);

/**
 * @brief Check if an interrupt has been triggered
 * 
 * @param dev Device to check
 * @param type Interrupt type to check
 * @return true if the interrupt is active, false otherwise
 */
bool sensor_node_interrupt_is_triggered(const struct device *dev, enum interrupt_type type);

/**
 * @brief Clear an interrupt
 * 
 * @param dev Device to clear
 * @param type Interrupt type to clear
 * @return 0 if successful, negative error code otherwise
 */
int sensor_node_clear_interrupt(const struct device *dev, enum interrupt_type type);

/**
 * @brief Get the last error code from the sensor node
 * 
 * @param dev Device to check
 * @return Error code or 0 if no error
 */
uint8_t sensor_node_get_last_error(const struct device *dev);

#endif /* SENSOR_NODE_DRIVER_H */
