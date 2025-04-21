/**
 * @file sensors.h
 * @brief Sensor interface for the sensor node
 */

#ifndef SENSORS_H
#define SENSORS_H

#include <stdbool.h>
#include <stdint.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/adc.h"

// I2C addresses
#define TSL2591_I2C_ADDR 0x29
#define SOIL_SENSOR_I2C_ADDR 0x36

// TSL2591 registers
#define TSL2591_COMMAND_BIT (0xA0)
#define TSL2591_ENABLE_REGISTER (0x00)
#define TSL2591_CONTROL_REGISTER (0x01)
#define TSL2591_ID_REGISTER (0x12)
#define TSL2591_STATUS_REGISTER (0x13)
#define TSL2591_CHAN0_LOW (0x14)
#define TSL2591_CHAN0_HIGH (0x15)
#define TSL2591_CHAN1_LOW (0x16)
#define TSL2591_CHAN1_HIGH (0x17)

// TSL2591 constants
#define TSL2591_ENABLE_POWERON (0x01)
#define TSL2591_ENABLE_AEN (0x02)
#define TSL2591_GAIN_LOW (0x00)
#define TSL2591_GAIN_MED (0x10)
#define TSL2591_GAIN_HIGH (0x20)
#define TSL2591_GAIN_MAX (0x30)
#define TSL2591_INTEGRATIONTIME_100MS (0x00)
#define TSL2591_INTEGRATIONTIME_200MS (0x01)
#define TSL2591_INTEGRATIONTIME_300MS (0x02)
#define TSL2591_INTEGRATIONTIME_400MS (0x03)
#define TSL2591_INTEGRATIONTIME_500MS (0x04)
#define TSL2591_INTEGRATIONTIME_600MS (0x05)

// Soil sensor registers (seesaw)
#define SEESAW_STATUS_BASE (0x00)
#define SEESAW_TOUCH_BASE (0x0F)
#define SEESAW_STATUS_HW_ID (0x01)
#define SEESAW_STATUS_TEMP (0x04)
#define SEESAW_TOUCH_CHANNEL_OFFSET (0x10)

// Soil sensor constants
#define SEESAW_HW_ID_CODE (0x55)

// Temperature sensor constants
#define TEMP_SENSOR_ADC_CHANNEL 0
#define TEMP_SENSOR_MV_PER_C 10.0f  // MCP9700 produces 10mV per °C
#define TEMP_SENSOR_OFFSET_MV 500.0f // 500mV at 0°C

// I2C interface
#define I2C_PORT i2c0
#define I2C_SDA_PIN 0
#define I2C_SCL_PIN 1
#define I2C_FREQ 100000

// ADC interface
#define ADC_PIN 26  // GPIO26 is ADC0

// Sensor data structure
typedef struct {
    float light;             // Light level in lux
    float temperature;       // Temperature in Celsius
    float soil_moisture;     // Soil moisture (0-100%)
    bool valid_light;        // Flag indicating if light data is valid
    bool valid_temperature;  // Flag indicating if temperature data is valid
    bool valid_soil;         // Flag indicating if soil moisture data is valid
} sensor_data_t;

// Configuration structure
typedef struct {
    float temp_high_threshold;    // High temperature threshold in Celsius
    float soil_low_threshold;     // Low soil moisture threshold in percentage
    float light_low_threshold;    // Low light threshold in lux
    float light_high_threshold;   // High light threshold in lux
    uint32_t sampling_rate_ms;    // Sampling rate in milliseconds
    bool int_enable_temp_high;    // Enable high temperature interrupt
    bool int_enable_soil_low;     // Enable low soil moisture interrupt
    bool int_enable_light_low;    // Enable low light interrupt
    bool int_enable_light_high;   // Enable high light interrupt
} sensor_config_t;

// Interrupt status structure
typedef struct {
    bool temp_high;         // High temperature detected
    bool soil_low;          // Low soil moisture detected
    bool light_low;         // Low light detected
    bool light_high;        // High light detected
} interrupt_status_t;

/**
 * @brief Initialize all sensors
 * 
 * @return true if all sensors initialized successfully, false otherwise
 */
bool sensors_init(void);

/**
 * @brief Read data from all sensors
 * 
 * @param data Pointer to sensor_data_t structure to store data
 * @return true if at least one sensor read successfully, false if all failed
 */
bool sensors_read(sensor_data_t *data);

/**
 * @brief Set sensor configuration
 * 
 * @param config Pointer to configuration structure
 * @return true if configuration was set successfully, false otherwise
 */
bool sensors_set_config(const sensor_config_t *config);

/**
 * @brief Get current sensor configuration
 * 
 * @param config Pointer to configuration structure to fill
 * @return true if configuration was retrieved successfully, false otherwise
 */
bool sensors_get_config(sensor_config_t *config);

/**
 * @brief Check for interrupt conditions
 * 
 * @param data Current sensor data
 * @param status Pointer to interrupt_status_t structure to fill
 * @return true if any interrupt condition is active, false otherwise
 */
bool sensors_check_interrupts(const sensor_data_t *data, interrupt_status_t *status);

#endif /* SENSORS_H */
