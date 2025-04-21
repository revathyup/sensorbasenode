/**
 * @file tsl2591.c
 * @brief Driver implementation for TSL2591 light sensor
 */
#include "tsl2591.h"
#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"

// Static variables
static i2c_inst_t *i2c = NULL;
static tsl2591_gain_t current_gain = TSL2591_GAIN_MED;
static tsl2591_integration_time_t current_integration_time = TSL2591_INTEGRATIONTIME_100MS;

/**
 * @brief Write a byte to a register
 * 
 * @param reg_addr Register address
 * @param data Data to write
 * @return true if successful, false otherwise
 */
static bool tsl2591_write_reg(uint8_t reg_addr, uint8_t data) {
    uint8_t buffer[2];
    buffer[0] = TSL2591_COMMAND_BIT | reg_addr;
    buffer[1] = data;
    
    int bytes_written = i2c_write_blocking(i2c, TSL2591_I2C_ADDR, buffer, 2, false);
    return bytes_written == 2;
}

/**
 * @brief Read a byte from a register
 * 
 * @param reg_addr Register address
 * @param data Pointer to store read data
 * @return true if successful, false otherwise
 */
static bool tsl2591_read_reg(uint8_t reg_addr, uint8_t *data) {
    uint8_t cmd = TSL2591_COMMAND_BIT | reg_addr;
    
    if (i2c_write_blocking(i2c, TSL2591_I2C_ADDR, &cmd, 1, true) != 1) {
        return false;
    }
    
    if (i2c_read_blocking(i2c, TSL2591_I2C_ADDR, data, 1, false) != 1) {
        return false;
    }
    
    return true;
}

/**
 * @brief Initialize the TSL2591 sensor
 * 
 * @param i2c_bus I2C bus instance
 * @return true if initialization successful, false otherwise
 */
bool tsl2591_init(i2c_inst_t *i2c_bus) {
    i2c = i2c_bus;
    uint8_t device_id;
    
    // Check if sensor is connected by reading device ID
    if (!tsl2591_read_reg(TSL2591_DEVICE_ID, &device_id)) {
        printf("TSL2591: Failed to read device ID\n");
        return false;
    }
    
    if (device_id != 0x50) {
        printf("TSL2591: Invalid device ID: 0x%02X\n", device_id);
        return false;
    }
    
    // Reset the device
    if (!tsl2591_write_reg(TSL2591_CONTROL_REGISTER, TSL2591_SRESET)) {
        printf("TSL2591: Failed to reset\n");
        return false;
    }
    
    // Wait for reset to complete
    sleep_ms(100);
    
    // Enable the device
    if (!tsl2591_write_reg(TSL2591_ENABLE_REGISTER, TSL2591_ENABLE_PON | TSL2591_ENABLE_AEN)) {
        printf("TSL2591: Failed to enable\n");
        return false;
    }
    
    // Set default gain and integration time
    tsl2591_set_gain(current_gain);
    tsl2591_set_integration_time(current_integration_time);
    
    return true;
}

/**
 * @brief Set the gain of the TSL2591 sensor
 * 
 * @param gain Gain value to set
 */
void tsl2591_set_gain(tsl2591_gain_t gain) {
    current_gain = gain;
    uint8_t control_value = current_gain | current_integration_time;
    tsl2591_write_reg(TSL2591_CONTROL_REGISTER, control_value);
}

/**
 * @brief Set the integration time of the TSL2591 sensor
 * 
 * @param time Integration time to set
 */
void tsl2591_set_integration_time(tsl2591_integration_time_t time) {
    current_integration_time = time;
    uint8_t control_value = current_gain | current_integration_time;
    tsl2591_write_reg(TSL2591_CONTROL_REGISTER, control_value);
}

/**
 * @brief Get full luminosity (CH0 and CH1) from the sensor
 * 
 * @param full Pointer to store full luminosity value (CH0)
 * @param ir Pointer to store IR luminosity value (CH1)
 * @return true if successful, false otherwise
 */
bool tsl2591_get_full_luminosity(uint16_t *full, uint16_t *ir) {
    uint8_t c0_low, c0_high, c1_low, c1_high;
    
    // Wait for integration to complete based on integration time
    switch (current_integration_time) {
        case TSL2591_INTEGRATIONTIME_100MS: sleep_ms(120); break;
        case TSL2591_INTEGRATIONTIME_200MS: sleep_ms(220); break;
        case TSL2591_INTEGRATIONTIME_300MS: sleep_ms(320); break;
        case TSL2591_INTEGRATIONTIME_400MS: sleep_ms(420); break;
        case TSL2591_INTEGRATIONTIME_500MS: sleep_ms(520); break;
        case TSL2591_INTEGRATIONTIME_600MS: sleep_ms(620); break;
    }
    
    // Read channel 0 (full spectrum)
    if (!tsl2591_read_reg(TSL2591_C0DATAL_REGISTER, &c0_low) ||
        !tsl2591_read_reg(TSL2591_C0DATAH_REGISTER, &c0_high)) {
        return false;
    }
    
    // Read channel 1 (IR)
    if (!tsl2591_read_reg(TSL2591_C1DATAL_REGISTER, &c1_low) ||
        !tsl2591_read_reg(TSL2591_C1DATAH_REGISTER, &c1_high)) {
        return false;
    }
    
    *full = (c0_high << 8) | c0_low;
    *ir = (c1_high << 8) | c1_low;
    
    return true;
}

/**
 * @brief Calculate lux based on sensor readings
 * 
 * @param ch0 Channel 0 value
 * @param ch1 Channel 1 value
 * @return float Lux value
 */
float tsl2591_calculate_lux(uint16_t ch0, uint16_t ch1) {
    float atime, again;
    float cpl, lux;

    // Calculate integration time in seconds
    switch (current_integration_time) {
        case TSL2591_INTEGRATIONTIME_100MS: atime = 0.1; break;
        case TSL2591_INTEGRATIONTIME_200MS: atime = 0.2; break;
        case TSL2591_INTEGRATIONTIME_300MS: atime = 0.3; break;
        case TSL2591_INTEGRATIONTIME_400MS: atime = 0.4; break;
        case TSL2591_INTEGRATIONTIME_500MS: atime = 0.5; break;
        case TSL2591_INTEGRATIONTIME_600MS: atime = 0.6; break;
        default: atime = 0.1;
    }

    // Calculate gain value
    switch (current_gain) {
        case TSL2591_GAIN_LOW:  again = 1.0; break;
        case TSL2591_GAIN_MED:  again = 25.0; break;
        case TSL2591_GAIN_HIGH: again = 428.0; break;
        case TSL2591_GAIN_MAX:  again = 9876.0; break;
        default: again = 25.0;
    }

    // See datasheet formula
    cpl = (atime * again) / 408.0;
    lux = (((float)ch0 - (float)ch1)) * (1.0 - ((float)ch1 / (float)ch0)) / cpl;

    // Check for underflow/zero
    if (lux < 0) {
        lux = 0;
    }

    return lux;
}

/**
 * @brief Get visible light value (CH0 - CH1)
 * 
 * @return uint16_t Visible light value
 */
uint16_t tsl2591_get_visible_light(void) {
    uint16_t full, ir;
    
    if (!tsl2591_get_full_luminosity(&full, &ir)) {
        return 0;
    }
    
    // Ensure visible light doesn't underflow
    if (full <= ir) {
        return 0;
    }
    
    return full - ir;
}

/**
 * @brief Set interrupt threshold for the sensor
 * 
 * @param low Low threshold
 * @param high High threshold
 * @return true if successful, false otherwise
 */
bool tsl2591_set_interrupt_threshold(uint16_t low, uint16_t high) {
    if (!tsl2591_write_reg(TSL2591_AILTL_REGISTER, low & 0xFF) ||
        !tsl2591_write_reg(TSL2591_AILTH_REGISTER, low >> 8) ||
        !tsl2591_write_reg(TSL2591_AIHTL_REGISTER, high & 0xFF) ||
        !tsl2591_write_reg(TSL2591_AIHTH_REGISTER, high >> 8)) {
        return false;
    }

    return true;
}

/**
 * @brief Clear the interrupt flag
 * 
 * @return true if successful, false otherwise
 */
bool tsl2591_clear_interrupt(void) {
    uint8_t cmd = TSL2591_COMMAND_BIT | 0xE0; // Special command to clear interrupt
    return i2c_write_blocking(i2c, TSL2591_I2C_ADDR, &cmd, 1, false) == 1;
}

/**
 * @brief Enable or disable the interrupt
 * 
 * @param enable true to enable, false to disable
 * @return true if successful, false otherwise
 */
bool tsl2591_enable_interrupt(bool enable) {
    uint8_t enable_reg;
    
    // Read current enable register value
    if (!tsl2591_read_reg(TSL2591_ENABLE_REGISTER, &enable_reg)) {
        return false;
    }
    
    // Modify the AIEN bit
    if (enable) {
        enable_reg |= TSL2591_ENABLE_AIEN;
    } else {
        enable_reg &= ~TSL2591_ENABLE_AIEN;
    }
    
    // Write back to enable register
    return tsl2591_write_reg(TSL2591_ENABLE_REGISTER, enable_reg);
}
