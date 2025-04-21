/**
 * @file stemma_soil.h
 * @brief Driver for Adafruit STEMMA Soil Moisture Sensor
 */
#ifndef STEMMA_SOIL_H
#define STEMMA_SOIL_H

#include <stdint.h>
#include <stdbool.h>
#include "hardware/i2c.h"

// STEMMA Soil I2C address
#define STEMMA_SOIL_I2C_ADDR 0x36

// STEMMA Soil seesaw registers and commands
#define SEESAW_STATUS_BASE 0x00
#define SEESAW_TOUCH_BASE 0x0F
#define SEESAW_STATUS_TEMP 0x04
#define SEESAW_TOUCH_CHANNEL_OFFSET 0x10

// Function prototypes
bool stemma_soil_init(i2c_inst_t *i2c_bus);
uint16_t stemma_soil_get_moisture(void);
float stemma_soil_get_temperature(void);
bool stemma_soil_set_address(uint8_t new_addr);

#endif /* STEMMA_SOIL_H */
