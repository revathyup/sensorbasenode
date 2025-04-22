/**
 * @file sensor_node_driver.h
 * @brief Zephyr driver for interfacing with the RP2040 sensor node
 */

#ifndef SENSOR_NODE_DRIVER_H
#define SENSOR_NODE_DRIVER_H

#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/uart.h>
#include "../../protocol.h"  // Shared protocol definitions

/* Device config structure */
struct sensor_node_config {
    struct uart_config uart_cfg;
    uint8_t tx_pin;
    uint8_t rx_pin;
};

/* Driver data structure */
struct sensor_node_data {
    const struct device *uart_dev;
    struct k_mutex data_mutex;
    
    /* Sensor readings */
    float light_lux;
    uint16_t light_full;
    uint16_t light_ir;
    uint16_t light_visible;
    
    float temperature;
    
    uint16_t soil_moisture;
    uint8_t soil_moisture_pct;
    float soil_temperature;
    
    /* Alert thresholds */
    float light_low_threshold;
    float light_high_threshold;
    float temp_low_threshold;
    float temp_high_threshold;
    uint16_t soil_dry_threshold;
    uint16_t soil_wet_threshold;
    
    /* Communication buffer */
    protocol_packet_t rx_packet;
    uint8_t rx_buffer[sizeof(protocol_packet_t)];
    size_t rx_index;
    bool packet_ready;
};

/* Sensor channel IDs */
enum sensor_node_channel {
    SENSOR_NODE_CHANNEL_LIGHT_LUX = SENSOR_CHAN_LIGHT,
    SENSOR_NODE_CHANNEL_LIGHT_FULL = SENSOR_CHAN_PROX,  // Repurposing proximity channel for full spectrum
    SENSOR_NODE_CHANNEL_LIGHT_IR = SENSOR_CHAN_IR,
    SENSOR_NODE_CHANNEL_LIGHT_VISIBLE = SENSOR_CHAN_BLUE,  // Repurposing blue channel for visible light
    SENSOR_NODE_CHANNEL_TEMPERATURE = SENSOR_CHAN_AMBIENT_TEMP,
    SENSOR_NODE_CHANNEL_SOIL_MOISTURE = SENSOR_CHAN_HUMIDITY,  // Repurposing humidity for soil moisture
    SENSOR_NODE_CHANNEL_SOIL_TEMPERATURE = SENSOR_CHAN_DIE_TEMP,  // Repurposing die temp for soil temp
};

/* API function prototypes */

/**
 * @brief Initialize the sensor node driver
 *
 * @param dev Device instance
 * @return 0 if successful, negative errno code if failed
 */
int sensor_node_init(const struct device *dev);

/**
 * @brief Send a ping command to the sensor node
 *
 * @param dev Device instance
 * @return 0 if successful, negative errno code if failed
 */
int sensor_node_ping(const struct device *dev);

/**
 * @brief Request sensor data from the sensor node
 *
 * @param dev Device instance
 * @return 0 if successful, negative errno code if failed
 */
int sensor_node_fetch_data(const struct device *dev);

/**
 * @brief Set sensor threshold 
 *
 * @param dev Device instance
 * @param sensor_type Type of sensor (SENSOR_LIGHT, SENSOR_TEMPERATURE, SENSOR_SOIL_MOISTURE)
 * @param alert_type Type of alert (LOW, HIGH, DRY, WET)
 * @param threshold Threshold value
 * @return 0 if successful, negative errno code if failed
 */
int sensor_node_set_threshold(const struct device *dev, uint8_t sensor_type, 
                              uint8_t alert_type, float threshold);

/* Declaration for driver API structure */
extern const struct sensor_driver_api sensor_node_driver_api;

#endif /* SENSOR_NODE_DRIVER_H */