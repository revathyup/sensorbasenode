/**
 * @file mcp9700.h
 * @brief Driver for MCP9700/9700A temperature sensor
 */
#ifndef MCP9700_H
#define MCP9700_H

#include <stdint.h>
#include <stdbool.h>
#include "hardware/adc.h"

// MCP9700/9700A Constants
#define MCP9700_V0C_MV 500.0f   // 500mV at 0°C
#define MCP9700_TC_MV 10.0f     // 10mV/°C temperature coefficient

// Function prototypes
bool mcp9700_init(uint adc_pin);
float mcp9700_read_temperature(void);
uint16_t mcp9700_read_raw(void);
uint8_t mcp9700_get_adc_channel(void);

#endif /* MCP9700_H */
