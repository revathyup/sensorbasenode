/**
 * @file stemma_soil.h
 * @brief Driver interface for Adafruit STEMMA Soil Moisture Sensor
 */

#ifndef STEMMA_SOIL_H
#define STEMMA_SOIL_H

#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <stdbool.h>
#include <stdint.h>

// Default I2C address for the STEMMA Soil sensor
#define STEMMA_SOIL_I2C_ADDR 0x36

// SeeSaw register base addresses
#define SEESAW_STATUS_BASE 0x00
#define SEESAW_TOUCH_BASE 0x0F

// Register addresses
#define SEESAW_STATUS_TEMP 0x04
#define SEESAW_TOUCH_CHANNEL_OFFSET 0x10

/**
 * @brief Initialize the STEMMA Soil sensor
 * 
 * @param i2c_bus I2C device
 * @return true if initialization successful, false otherwise
 */
bool stemma_soil_init(const struct device *i2c_bus);

/**
 * @brief Get moisture level from the sensor
 * 
 * @return uint16_t Moisture level (0-1023)
 */
uint16_t stemma_soil_get_moisture(void);

/**
 * @brief Get temperature from the sensor
 * 
 * @return float Temperature in Celsius
 */
float stemma_soil_get_temperature(void);

/**
 * @brief Set a new I2C address for the sensor
 * 
 * @param new_addr New I2C address (0x01-0x7F)
 * @return true if successful, false otherwise
 */
bool stemma_soil_set_address(uint8_t new_addr);

#endif /* STEMMA_SOIL_H */