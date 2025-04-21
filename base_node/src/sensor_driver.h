/**
 * @file sensor_driver.h
 * @brief Zephyr driver for sensor node
 */
#ifndef SENSOR_DRIVER_H
#define SENSOR_DRIVER_H

#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/drivers/gpio.h>
#include "protocol.h"

// Sensor channels for our custom sensor node
enum sensor_node_channel {
    SENSOR_NODE_CHAN_LIGHT = SENSOR_CHAN_PRIV_START,
    SENSOR_NODE_CHAN_TEMPERATURE,
    SENSOR_NODE_CHAN_SOIL_MOISTURE
};

// Function prototypes
int sensor_node_init(const struct device *dev);
int sensor_node_sample_fetch(const struct device *dev, enum sensor_channel chan);
int sensor_node_channel_get(const struct device *dev, enum sensor_channel chan, struct sensor_value *val);
int sensor_node_attr_set(const struct device *dev, enum sensor_channel chan, enum sensor_attribute attr, const struct sensor_value *val);
int sensor_node_trigger_set(const struct device *dev, const struct sensor_trigger *trig, sensor_trigger_handler_t handler);
int sensor_node_send_command(const struct device *dev, uint8_t cmd, const uint8_t *data, uint8_t data_len);

// Configuration thresholds
int sensor_node_set_light_threshold(const struct device *dev, uint16_t threshold, uint8_t threshold_type);
int sensor_node_set_temperature_threshold(const struct device *dev, uint16_t threshold, uint8_t threshold_type);
int sensor_node_set_moisture_threshold(const struct device *dev, uint16_t threshold, uint8_t threshold_type);

#endif /* SENSOR_DRIVER_H */
