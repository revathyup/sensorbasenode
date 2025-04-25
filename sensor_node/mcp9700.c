/**
 * @file mcp9700.c
 * @brief Driver for the MCP9700 temperature sensor
 */

#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/adc.h"
#include "mcp9700.h"

// Static variables
static uint adc_channel;  // ADC channel number

/**
 * @brief Initialize the MCP9700 temperature sensor
 * 
 * @param adc_pin GPIO pin connected to the MCP9700 output
 * @return true if initialization successful, false otherwise
 */
bool mcp9700_init(uint adc_pin) {
    // Check if the pin is valid for ADC
    if (adc_pin < 26 || adc_pin > 29) {
        printf("MCP9700: Invalid ADC pin (must be GPIO 26-29)\n");
        return false;
    }
    
    // Initialize ADC
    adc_init();
    
    // Configure the pin for ADC use
    adc_gpio_init(adc_pin);
    
    // Calculate ADC channel number (ADC0 is GPIO26, ADC1 is GPIO27, etc.)
    adc_channel = adc_pin - 26;
    
    printf("MCP9700: Initialized successfully on ADC channel %d (GPIO%d)\n", 
           adc_channel, adc_pin);
    
    return true;
}

/**
 * @brief Read temperature from MCP9700 sensor
 * 
 * @return float Temperature in degrees Celsius
 */
float mcp9700_read_temperature(void) {
    // Select ADC channel
    adc_select_input(adc_channel);
    
    // Read ADC value (12-bit resolution, 0-4095)
    uint16_t adc_value = adc_read();
    
    // Convert ADC value to voltage
    // 3.3V reference voltage, 12-bit ADC (4096 steps)
    float voltage = (float)adc_value * 3.3f / 4096.0f;
    
    // Convert voltage to temperature
    // MCP9700 has 500mV (0.5V) output at 0°C
    // 10mV (0.01V) per degree Celsius
    float temperature = (voltage - 0.5f) / 0.01f;
    
    return temperature;
}