/**
 * @file stemma_soil.c
 * @brief Driver implementation for Adafruit STEMMA Soil Moisture Sensor
 */
#include "stemma_soil.h"
#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"

// Static variables
static i2c_inst_t *i2c = NULL;
static uint8_t soil_addr = STEMMA_SOIL_I2C_ADDR;

/**
 * @brief Write data to a STEMMA Soil register
 * 
 * @param reg_base Register base address
 * @param reg_addr Register offset
 * @param buf Data buffer to write
 * @param len Length of data buffer
 * @return true if successful, false otherwise
 */
static bool stemma_soil_write(uint8_t reg_base, uint8_t reg_addr, const uint8_t *buf, uint8_t len) {
    uint8_t *buffer = malloc(len + 2);
    if (!buffer) {
        return false;
    }
    
    buffer[0] = reg_base;
    buffer[1] = reg_addr;
    memcpy(buffer + 2, buf, len);
    
    int bytes_written = i2c_write_blocking(i2c, soil_addr, buffer, len + 2, false);
    free(buffer);
    
    return bytes_written == (len + 2);
}

/**
 * @brief Read data from a STEMMA Soil register
 * 
 * @param reg_base Register base address
 * @param reg_addr Register offset
 * @param buf Buffer to store read data
 * @param len Length of data to read
 * @return true if successful, false otherwise
 */
static bool stemma_soil_read(uint8_t reg_base, uint8_t reg_addr, uint8_t *buf, uint8_t len) {
    uint8_t reg[2] = {reg_base, reg_addr};
    
    if (i2c_write_blocking(i2c, soil_addr, reg, 2, true) != 2) {
        return false;
    }
    
    // Delay to give the sensor time to process
    sleep_ms(5);
    
    if (i2c_read_blocking(i2c, soil_addr, buf, len, false) != len) {
        return false;
    }
    
    return true;
}

/**
 * @brief Initialize the STEMMA Soil sensor
 * 
 * @param i2c_bus I2C bus instance
 * @return true if initialization successful, false otherwise
 */
bool stemma_soil_init(i2c_inst_t *i2c_bus) {
    i2c = i2c_bus;
    
    // Check if sensor is connected by reading a register
    uint8_t data[4];
    if (!stemma_soil_read(SEESAW_STATUS_BASE, SEESAW_STATUS_TEMP, data, 4)) {
        printf("STEMMA Soil: Failed to communicate with sensor\n");
        return false;
    }
    
    return true;
}

/**
 * @brief Get moisture level from the sensor
 * 
 * @return uint16_t Moisture level (0-1023)
 */
uint16_t stemma_soil_get_moisture(void) {
    uint8_t data[2];
    
    if (!stemma_soil_read(SEESAW_TOUCH_BASE, SEESAW_TOUCH_CHANNEL_OFFSET, data, 2)) {
        printf("STEMMA Soil: Failed to read moisture\n");
        return 0;
    }
    
    // Convert big-endian to little-endian
    return (data[0] << 8) | data[1];
}

/**
 * @brief Get temperature from the sensor
 * 
 * @return float Temperature in Celsius
 */
float stemma_soil_get_temperature(void) {
    uint8_t data[4];
    
    if (!stemma_soil_read(SEESAW_STATUS_BASE, SEESAW_STATUS_TEMP, data, 4)) {
        printf("STEMMA Soil: Failed to read temperature\n");
        return 0.0f;
    }
    
    // Convert big-endian to little-endian and scale (temperature is in 0.01 degrees C)
    uint32_t temp = ((uint32_t)data[0] << 24) | ((uint32_t)data[1] << 16) |
                   ((uint32_t)data[2] << 8) | data[3];
    
    return temp / 100.0f;
}

/**
 * @brief Set a new I2C address for the sensor
 * 
 * @param new_addr New I2C address (0x01-0x7F)
 * @return true if successful, false otherwise
 */
bool stemma_soil_set_address(uint8_t new_addr) {
    if (new_addr < 0x08 || new_addr > 0x77) {
        // Invalid I2C address
        return false;
    }
    
    uint8_t addr_data[1] = {new_addr};
    if (!stemma_soil_write(0x00, 0x00, addr_data, 1)) {
        return false;
    }
    
    // Update stored address
    soil_addr = new_addr;
    return true;
}
