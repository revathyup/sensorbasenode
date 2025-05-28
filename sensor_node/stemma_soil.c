/**
 * @file stemma_soil.c
 * @brief Driver for the STEMMA soil moisture sensor
 */

#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "stemma_soil.h"

// Static variables
static i2c_inst_t *i2c_instance;

/**
 * @brief Read a 16-bit register from the STEMMA sensor
 * 
 * @param reg Register address
 * @param value Pointer to store the read value
 * @return true if successful, false otherwise
 */
static bool stemma_soil_read_register(uint8_t reg, uint16_t *value) {
    uint8_t data[2];
    
    // Write the register address
    int result = i2c_write_blocking(i2c_instance, STEMMA_SOIL_I2C_ADDR, &reg, 1, true);
    if (result != 1) {
        return false;
    }
    
    // Read the 16-bit response
    result = i2c_read_blocking(i2c_instance, STEMMA_SOIL_I2C_ADDR, data, 2, false);
    if (result != 2) {
        return false;
    }
    
    // Combine the bytes (MSB first)
    *value = (data[0] << 8) | data[1];
    
    return true;
}

/**
 * @brief Initialize the STEMMA soil moisture sensor
 * 
 * @param i2c I2C instance to use
 * @return true if initialization successful, false otherwise
 */
bool stemma_soil_init(i2c_inst_t *i2c) {
    i2c_instance = i2c;
    
    // Try to read the status register to check if the sensor is responding
    uint16_t status;
    if (!stemma_soil_read_register(STEMMA_REG_STATUS, &status)) {
        //printf("STEMMA: Failed to read sensor status\n");
        return false;
    }
    
    //printf("STEMMA: Initialized successfully (status: 0x%04X)\n", status);
    return true;
}

/**
 * @brief Read soil moisture from STEMMA sensor
 * 
 * @param moisture Pointer to store moisture reading (0-1000)
 * @return true if successful, false otherwise
 */
bool stemma_soil_read_moisture(uint16_t *moisture) {
    // Read touchpad 0 (capacitive moisture reading)
    uint16_t raw_value;
    if (!stemma_soil_read_register(STEMMA_REG_TOUCH_CHANNEL_0, &raw_value)) {
        return false;
    }
    
    // STEMMA soil moisture value ranges from approximately
    // 200 (very dry) to 2000 (very wet)
    // Scale to 0-1000 for more intuitive values
    
    // Clamp raw value to 200-2000 range
    if (raw_value < 200) raw_value = 200;
    if (raw_value > 2000) raw_value = 2000;
    
    // Map from 200-2000 to 0-1000
    *moisture = (raw_value - 200) * 1000 / 1800;
    
    return true;
}

/**
 * @brief Read temperature from STEMMA sensor (if available)
 * 
 * @param temperature Pointer to store temperature in degrees Celsius
 * @return true if successful, false otherwise
 */
bool stemma_soil_read_temperature(float *temperature) {
    // Read temperature register
    uint16_t raw_temp;
    if (!stemma_soil_read_register(STEMMA_REG_TEMP_BASE, &raw_temp)) {
        return false;
    }
    
    // Convert raw value to temperature in Celsius
    // According to STEMMA documentation
    *temperature = raw_temp / 10.0f;
    
    return true;
}