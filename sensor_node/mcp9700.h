/**
 * @file mcp9700.h
 * @brief Driver for the MCP9700 temperature sensor
 */

#ifndef MCP9700_H
#define MCP9700_H

#include <stdbool.h>
#include <stdint.h>
#include "hardware/adc.h"

/**
 * @brief Initialize the MCP9700 temperature sensor
 * 
 * @param adc_pin GPIO pin connected to the MCP9700 output
 * @return true if initialization successful, false otherwise
 */
bool mcp9700_init(uint adc_pin);

/**
 * @brief Read temperature from MCP9700 sensor
 * 
 * @return float Temperature in degrees Celsius
 */
float mcp9700_read_temperature(void);

/**
 * @brief Structure for temperature data
 */
typedef struct {
    uint8_t sensor_type;  // Always SENSOR_TEMPERATURE
    float temperature;    // Temperature in degrees Celsius
} temperature_data_t;

#endif /* MCP9700_H */