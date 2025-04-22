/**
 * @file tsl2591.c
 * @brief Driver implementation for the TSL2591 light sensor
 */
#include <stdio.h>
#include "pico/stdlib.h"
#include "tsl2591.h"

// Static variables
static i2c_inst_t *i2c_instance;
static tsl2591_gain_t current_gain = TSL2591_GAIN_MED;
static tsl2591_integration_time_t current_integration_time = TSL2591_INTEGRATIONTIME_300MS;

// Integration time conversion table (in milliseconds)
static const uint16_t integration_time_ms[] = {
    100,  // TSL2591_INTEGRATIONTIME_100MS
    200,  // TSL2591_INTEGRATIONTIME_200MS
    300,  // TSL2591_INTEGRATIONTIME_300MS
    400,  // TSL2591_INTEGRATIONTIME_400MS
    500,  // TSL2591_INTEGRATIONTIME_500MS
    600,  // TSL2591_INTEGRATIONTIME_600MS
};

/**
 * @brief Write to a TSL2591 register
 * 
 * @param reg Register to write to
 * @param value Value to write
 * @return true if successful, false otherwise
 */
static bool tsl2591_write_reg(uint8_t reg, uint8_t value) {
    uint8_t data[2];
    data[0] = TSL2591_COMMAND_BIT | reg;
    data[1] = value;
    
    int bytes_written = i2c_write_blocking(i2c_instance, TSL2591_I2C_ADDR, data, 2, false);
    return (bytes_written == 2);
}

/**
 * @brief Read from a TSL2591 register
 * 
 * @param reg Register to read from
 * @param value Pointer to store the read value
 * @return true if successful, false otherwise
 */
static bool tsl2591_read_reg(uint8_t reg, uint8_t *value) {
    uint8_t cmd = TSL2591_COMMAND_BIT | reg;
    
    if (i2c_write_blocking(i2c_instance, TSL2591_I2C_ADDR, &cmd, 1, true) != 1) {
        return false;
    }
    
    if (i2c_read_blocking(i2c_instance, TSL2591_I2C_ADDR, value, 1, false) != 1) {
        return false;
    }
    
    return true;
}

/**
 * @brief Initialize the TSL2591 sensor
 * 
 * @param i2c I2C instance to use
 * @return true if initialization successful, false otherwise
 */
bool tsl2591_init(i2c_inst_t *i2c) {
    i2c_instance = i2c;
    
    // Check sensor ID
    uint8_t id;
    if (!tsl2591_read_reg(TSL2591_REGISTER_ID, &id)) {
        printf("TSL2591: Failed to read sensor ID\n");
        return false;
    }
    
    // Expected ID is 0x50
    if (id != 0x50) {
        printf("TSL2591: Unexpected sensor ID: 0x%02X\n", id);
        return false;
    }
    
    // Disable the sensor initially
    if (!tsl2591_disable()) {
        printf("TSL2591: Failed to disable sensor\n");
        return false;
    }
    
    // Set default gain and integration time
    if (!tsl2591_set_gain(current_gain)) {
        printf("TSL2591: Failed to set gain\n");
        return false;
    }
    
    if (!tsl2591_set_integration_time(current_integration_time)) {
        printf("TSL2591: Failed to set integration time\n");
        return false;
    }
    
    printf("TSL2591: Initialized successfully\n");
    return true;
}

/**
 * @brief Set the gain for the TSL2591 sensor
 * 
 * @param gain Gain setting
 * @return true if successful, false otherwise
 */
bool tsl2591_set_gain(tsl2591_gain_t gain) {
    // Read current control register value
    uint8_t ctrl_reg;
    if (!tsl2591_read_reg(TSL2591_REGISTER_CONTROL, &ctrl_reg)) {
        printf("TSL2591: Failed to read control register\n");
        return false;
    }
    
    // Clear gain bits and set new gain
    ctrl_reg &= 0xCF; // Clear bits 4-5
    ctrl_reg |= gain;
    
    // Write updated control register
    if (!tsl2591_write_reg(TSL2591_REGISTER_CONTROL, ctrl_reg)) {
        printf("TSL2591: Failed to write control register\n");
        return false;
    }
    
    // Save current gain
    current_gain = gain;
    
    return true;
}

/**
 * @brief Set the integration time for the TSL2591 sensor
 * 
 * @param integration_time Integration time setting
 * @return true if successful, false otherwise
 */
bool tsl2591_set_integration_time(tsl2591_integration_time_t integration_time) {
    // Read current control register value
    uint8_t ctrl_reg;
    if (!tsl2591_read_reg(TSL2591_REGISTER_CONTROL, &ctrl_reg)) {
        printf("TSL2591: Failed to read control register\n");
        return false;
    }
    
    // Clear integration time bits and set new time
    ctrl_reg &= 0xF8; // Clear bits 0-2
    ctrl_reg |= integration_time;
    
    // Write updated control register
    if (!tsl2591_write_reg(TSL2591_REGISTER_CONTROL, ctrl_reg)) {
        printf("TSL2591: Failed to write control register\n");
        return false;
    }
    
    // Save current integration time
    current_integration_time = integration_time;
    
    return true;
}

/**
 * @brief Enable the TSL2591 sensor
 * 
 * @return true if successful, false otherwise
 */
bool tsl2591_enable(void) {
    // Enable power and ADC
    uint8_t enable_cmd = TSL2591_ENABLE_POWERON | TSL2591_ENABLE_AEN;
    if (!tsl2591_write_reg(TSL2591_REGISTER_ENABLE, enable_cmd)) {
        printf("TSL2591: Failed to enable sensor\n");
        return false;
    }
    
    return true;
}

/**
 * @brief Disable the TSL2591 sensor
 * 
 * @return true if successful, false otherwise
 */
bool tsl2591_disable(void) {
    // Disable all features
    if (!tsl2591_write_reg(TSL2591_REGISTER_ENABLE, 0x00)) {
        printf("TSL2591: Failed to disable sensor\n");
        return false;
    }
    
    return true;
}

/**
 * @brief Get integration time in milliseconds
 * 
 * @return uint16_t Integration time in milliseconds
 */
static uint16_t tsl2591_get_integration_time_ms(void) {
    // Map the integration time enum to actual milliseconds
    if (current_integration_time <= TSL2591_INTEGRATIONTIME_600MS) {
        return integration_time_ms[current_integration_time];
    } else {
        return integration_time_ms[0]; // Default to 100ms
    }
}

/**
 * @brief Read the full luminosity (visible + IR) from the sensor
 * 
 * @param channel0 Pointer to store full spectrum reading
 * @param channel1 Pointer to store infrared reading
 * @return true if successful, false otherwise
 */
bool tsl2591_get_full_luminosity(uint16_t *channel0, uint16_t *channel1) {
    // Enable the sensor
    if (!tsl2591_enable()) {
        printf("TSL2591: Failed to enable sensor for reading\n");
        return false;
    }
    
    // Wait for integration to complete
    sleep_ms(tsl2591_get_integration_time_ms() + 10);
    
    // Read the channels
    uint8_t chan0_low, chan0_high, chan1_low, chan1_high;
    
    if (!tsl2591_read_reg(TSL2591_REGISTER_CHAN0_L, &chan0_low) ||
        !tsl2591_read_reg(TSL2591_REGISTER_CHAN0_H, &chan0_high) ||
        !tsl2591_read_reg(TSL2591_REGISTER_CHAN1_L, &chan1_low) ||
        !tsl2591_read_reg(TSL2591_REGISTER_CHAN1_H, &chan1_high)) {
        printf("TSL2591: Failed to read channels\n");
        return false;
    }
    
    // Disable the sensor
    if (!tsl2591_disable()) {
        printf("TSL2591: Failed to disable sensor after reading\n");
        return false;
    }
    
    // Calculate channel values
    *channel0 = (chan0_high << 8) | chan0_low;
    *channel1 = (chan1_high << 8) | chan1_low;
    
    return true;
}

/**
 * @brief Get visible light reading
 * 
 * @return uint16_t Visible light reading (full - IR)
 */
uint16_t tsl2591_get_visible_light(void) {
    uint16_t ch0, ch1;
    
    if (!tsl2591_get_full_luminosity(&ch0, &ch1)) {
        return 0;
    }
    
    // Visible light = Full spectrum - Infrared
    if (ch0 > ch1) {
        return ch0 - ch1;
    } else {
        return 0;
    }
}

/**
 * @brief Calculate lux based on sensor readings
 * 
 * @param ch0 Full spectrum reading
 * @param ch1 Infrared reading
 * @return float Lux value
 */
float tsl2591_calculate_lux(uint16_t ch0, uint16_t ch1) {
    // Get integration time in milliseconds
    uint16_t integration_time = tsl2591_get_integration_time_ms();
    
    // Calculate gain factor
    float gain_factor;
    switch (current_gain) {
        case TSL2591_GAIN_1X:
            gain_factor = 1.0f;
            break;
        case TSL2591_GAIN_25X:
            gain_factor = 25.0f;
            break;
        case TSL2591_GAIN_428X:
            gain_factor = 428.0f;
            break;
        case TSL2591_GAIN_9876X:
            gain_factor = 9876.0f;
            break;
        default:
            gain_factor = 1.0f;
            break;
    }
    
    // Check if sensor saturated
    if ((ch0 == 0xFFFF) || (ch1 == 0xFFFF)) {
        return 0.0f;
    }
    
    // Calculate lux
    float atime = integration_time / 100.0f;  // Convert to seconds
    float cpl = (atime * gain_factor) / 408.0f;
    float lux = ((float)ch0 - (float)ch1) / cpl;
    
    // Ensure lux is not negative
    if (lux < 0) {
        lux = 0;
    }
    
    return lux;
}