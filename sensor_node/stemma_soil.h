/**
 * @file stemma_soil.h
 * @brief Driver for the STEMMA soil moisture sensor
 */

#ifndef STEMMA_SOIL_H
#define STEMMA_SOIL_H

#include <stdbool.h>
#include <stdint.h>
#include "hardware/i2c.h"

// STEMMA soil moisture sensor I2C address
#define STEMMA_SOIL_I2C_ADDR 0x36

// STEMMA register addresses
#define STEMMA_REG_STATUS          0x00
#define STEMMA_REG_TOUCH_CHANNEL_0 0x0F
#define STEMMA_REG_TOUCH_CHANNEL_1 0x10
#define STEMMA_REG_TOUCH_CHANNEL_2 0x11
#define STEMMA_REG_TOUCH_CHANNEL_3 0x12
#define STEMMA_REG_TEMP_BASE       0x13

/**
 * @brief Initialize the STEMMA soil moisture sensor
 * 
 * @param i2c I2C instance to use
 * @return true if initialization successful, false otherwise
 */
bool stemma_soil_init(i2c_inst_t *i2c);

/**
 * @brief Read soil moisture from STEMMA sensor
 * 
 * @param moisture Pointer to store moisture reading (0-1000)
 * @return true if successful, false otherwise
 */
bool stemma_soil_read_moisture(uint16_t *moisture);

/**
 * @brief Read temperature from STEMMA sensor (if available)
 * 
 * @param temperature Pointer to store temperature in degrees Celsius
 * @return true if successful, false otherwise
 */
bool stemma_soil_read_temperature(float *temperature);

/**
 * @brief Structure for soil moisture data
 */
typedef struct {
    uint8_t sensor_type;  // Always SENSOR_SOIL_MOISTURE
    uint16_t moisture;    // Moisture reading (0-1000)
    float temperature;    // Temperature in degrees Celsius (if available)
} soil_data_t;

#endif /* STEMMA_SOIL_H */