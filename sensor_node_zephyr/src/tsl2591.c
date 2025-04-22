/**
 * @file tsl2591.c
 * @brief Driver implementation for AMS TSL2591 light sensor
 */
#include "tsl2591.h"
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <string.h>
#include <math.h>

LOG_MODULE_REGISTER(tsl2591, CONFIG_LOG_DEFAULT_LEVEL);

// Static variables
static const struct device *i2c_dev = NULL;
static tsl2591_gain_t current_gain = TSL2591_GAIN_MED;
static tsl2591_integration_time_t current_integration_time = TSL2591_INTEGRATIONTIME_300MS;

// Conversion helpers for integration time
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
    
    return i2c_write(i2c_dev, data, 2, TSL2591_I2C_ADDR) == 0;
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
    
    if (i2c_write(i2c_dev, &cmd, 1, TSL2591_I2C_ADDR) != 0) {
        return false;
    }
    
    if (i2c_read(i2c_dev, value, 1, TSL2591_I2C_ADDR) != 0) {
        return false;
    }
    
    return true;
}

/**
 * @brief Initialize the TSL2591 light sensor
 * 
 * @param i2c_bus I2C device
 * @return true if initialization successful, false otherwise
 */
bool tsl2591_init(const struct device *i2c_bus) {
    i2c_dev = i2c_bus;
    
    if (!device_is_ready(i2c_dev)) {
        LOG_ERR("TSL2591: I2C device not ready");
        return false;
    }
    
    // Check sensor ID
    uint8_t id;
    if (!tsl2591_read_reg(TSL2591_REGISTER_ID, &id)) {
        LOG_ERR("TSL2591: Failed to read sensor ID");
        return false;
    }
    
    // Expected ID is 0x50
    if (id != 0x50) {
        LOG_ERR("TSL2591: Unexpected sensor ID: 0x%02X", id);
        return false;
    }
    
    // Disable the sensor initially
    if (!tsl2591_disable()) {
        LOG_ERR("TSL2591: Failed to disable sensor");
        return false;
    }
    
    // Set default gain and integration time
    if (!tsl2591_set_gain(current_gain)) {
        LOG_ERR("TSL2591: Failed to set gain");
        return false;
    }
    
    if (!tsl2591_set_integration_time(current_integration_time)) {
        LOG_ERR("TSL2591: Failed to set integration time");
        return false;
    }
    
    LOG_INF("TSL2591: Initialized successfully");
    return true;
}

/**
 * @brief Set the gain for the TSL2591 sensor
 * 
 * @param gain Gain setting to use
 * @return true if successful, false otherwise
 */
bool tsl2591_set_gain(tsl2591_gain_t gain) {
    // Read current control register value
    uint8_t ctrl_reg;
    if (!tsl2591_read_reg(TSL2591_REGISTER_CONTROL, &ctrl_reg)) {
        LOG_ERR("TSL2591: Failed to read control register");
        return false;
    }
    
    // Clear gain bits and set new gain
    ctrl_reg &= 0xCF; // Clear bits 4-5
    ctrl_reg |= gain;
    
    // Write updated control register
    if (!tsl2591_write_reg(TSL2591_REGISTER_CONTROL, ctrl_reg)) {
        LOG_ERR("TSL2591: Failed to write control register");
        return false;
    }
    
    // Save current gain
    current_gain = gain;
    
    LOG_DBG("TSL2591: Gain set to 0x%02X", gain);
    return true;
}

/**
 * @brief Set the integration time for the TSL2591 sensor
 * 
 * @param integration_time Integration time setting to use
 * @return true if successful, false otherwise
 */
bool tsl2591_set_integration_time(tsl2591_integration_time_t integration_time) {
    // Read current control register value
    uint8_t ctrl_reg;
    if (!tsl2591_read_reg(TSL2591_REGISTER_CONTROL, &ctrl_reg)) {
        LOG_ERR("TSL2591: Failed to read control register");
        return false;
    }
    
    // Clear integration time bits and set new time
    ctrl_reg &= 0xF8; // Clear bits 0-2
    ctrl_reg |= integration_time;
    
    // Write updated control register
    if (!tsl2591_write_reg(TSL2591_REGISTER_CONTROL, ctrl_reg)) {
        LOG_ERR("TSL2591: Failed to write control register");
        return false;
    }
    
    // Save current integration time
    current_integration_time = integration_time;
    
    LOG_DBG("TSL2591: Integration time set to 0x%02X", integration_time);
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
        LOG_ERR("TSL2591: Failed to enable sensor");
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
        LOG_ERR("TSL2591: Failed to disable sensor");
        return false;
    }
    
    return true;
}

/**
 * @brief Read the full luminosity (visible + IR) from the sensor
 * 
 * @param full Pointer to store the full spectrum reading
 * @param ir Pointer to store the infrared reading
 * @return true if reading successful, false otherwise
 */
bool tsl2591_get_full_luminosity(uint16_t *full, uint16_t *ir) {
    // Enable the sensor
    if (!tsl2591_enable()) {
        LOG_ERR("TSL2591: Failed to enable sensor for reading");
        return false;
    }
    
    // Wait for integration to complete
    k_msleep(tsl2591_get_integration_time_ms() + 10);
    
    // Read the channels
    uint8_t chan0_low, chan0_high, chan1_low, chan1_high;
    
    if (!tsl2591_read_reg(TSL2591_REGISTER_CHAN0_L, &chan0_low) ||
        !tsl2591_read_reg(TSL2591_REGISTER_CHAN0_H, &chan0_high) ||
        !tsl2591_read_reg(TSL2591_REGISTER_CHAN1_L, &chan1_low) ||
        !tsl2591_read_reg(TSL2591_REGISTER_CHAN1_H, &chan1_high)) {
        LOG_ERR("TSL2591: Failed to read channels");
        return false;
    }
    
    // Disable the sensor
    if (!tsl2591_disable()) {
        LOG_ERR("TSL2591: Failed to disable sensor after reading");
        return false;
    }
    
    // Calculate channel values
    uint16_t channel0 = (chan0_high << 8) | chan0_low;
    uint16_t channel1 = (chan1_high << 8) | chan1_low;
    
    // Channel 0 = Visible + IR, Channel 1 = IR only
    *full = channel0;
    *ir = channel1;
    
    LOG_DBG("TSL2591: Full spectrum: %u, IR: %u", *full, *ir);
    return true;
}

/**
 * @brief Calculate lux based on sensor readings
 * 
 * @param full_spectrum Full spectrum reading
 * @param ir_spectrum Infrared reading
 * @return float Calculated lux value
 */
float tsl2591_calculate_lux(uint16_t full_spectrum, uint16_t ir_spectrum) {
    // Get the current integration time in ms
    uint16_t integration_time_val = tsl2591_get_integration_time_ms();
    
    // Calculate gain factor
    float gain_val;
    switch (current_gain) {
        case TSL2591_GAIN_LOW:
            gain_val = 1.0f;
            break;
        case TSL2591_GAIN_MED:
            gain_val = 25.0f;
            break;
        case TSL2591_GAIN_HIGH:
            gain_val = 428.0f;
            break;
        case TSL2591_GAIN_MAX:
            gain_val = 9876.0f;
            break;
        default:
            gain_val = 1.0f;
            break;
    }
    
    // Check if either sensor is saturated
    if ((full_spectrum == 0xFFFF) || (ir_spectrum == 0xFFFF)) {
        // Sensor is saturated, reduce the gain or integration time
        LOG_WRN("TSL2591: Sensor saturated");
        return 0.0f;
    }
    
    // Avoid division by zero
    if (full_spectrum == 0) {
        return 0.0f;
    }
    
    // Visible light calculation
    float atime, again;
    float cpl, lux;
    
    // Integration time factor (atime) = integration_time_ms / 100
    atime = integration_time_val / 100.0f;
    
    // Gain factor (again) is already calculated above as gain_val
    again = gain_val;
    
    // Calculate counts per lux (cpl)
    cpl = (atime * again) / 408.0f;
    
    // Calculate lux using the formula from the datasheet
    lux = ((float)full_spectrum - (float)ir_spectrum) / cpl;
    
    // Ensure lux is not negative
    if (lux < 0) {
        lux = 0;
    }
    
    return lux;
}

/**
 * @brief Get integration time in milliseconds
 * 
 * @return uint16_t Integration time in milliseconds
 */
uint16_t tsl2591_get_integration_time_ms(void) {
    // Map the integration time enum to actual milliseconds
    if (current_integration_time <= TSL2591_INTEGRATIONTIME_600MS) {
        return integration_time_ms[current_integration_time];
    } else {
        return integration_time_ms[0]; // Default to 100ms
    }
}