/**
 * @file mcp9700.h
 * @brief Driver interface for MCP9700 temperature sensor
 */

#ifndef MCP9700_H
#define MCP9700_H

#include <zephyr/device.h>
#include <zephyr/drivers/adc.h>
#include <stdbool.h>
#include <stdint.h>

/**
 * @brief Initialize the MCP9700 temperature sensor
 * 
 * @param adc_dev ADC device
 * @param channel ADC channel
 * @return true if initialization successful, false otherwise
 */
bool mcp9700_init(const struct device *adc_dev, uint8_t channel);

/**
 * @brief Read temperature from the MCP9700 sensor
 * 
 * @return float Temperature in Celsius
 */
float mcp9700_read_temperature(void);

#endif /* MCP9700_H */