/**
 * @file mcp9700.c
 * @brief Driver implementation for MCP9700/9700A temperature sensor
 */
#include "mcp9700.h"
#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/adc.h"
#include "hardware/gpio.h"

// Static variables
static uint8_t adc_channel;
static float adc_voltage_scale;

/**
 * @brief Initialize the MCP9700/9700A temperature sensor
 * 
 * @param adc_pin ADC pin connected to the sensor
 * @return true if initialization successful, false otherwise
 */
bool mcp9700_init(uint adc_pin) {
    // Initialize ADC if not already done
    adc_init();
    
    // Calculate ADC channel from pin number
    adc_channel = adc_pin - 26;
    
    // Check if it's a valid ADC pin
    if (adc_channel > 3) {
        printf("MCP9700: Invalid ADC pin %u\n", adc_pin);
        return false;
    }
    
    // Configure ADC pin
    adc_gpio_init(adc_pin);
    
    // Calculate voltage scale (12-bit ADC, 3.3V reference)
    adc_voltage_scale = 3.3f / (1 << 12);
    
    return true;
}

/**
 * @brief Read raw ADC value from the sensor
 * 
 * @return uint16_t Raw ADC value (0-4095)
 */
uint16_t mcp9700_read_raw(void) {
    // Select ADC channel
    adc_select_input(adc_channel);
    
    // Average multiple readings for stability
    const int num_samples = 10;
    uint32_t sum = 0;
    
    for (int i = 0; i < num_samples; i++) {
        sum += adc_read();
        sleep_ms(1);
    }
    
    return sum / num_samples;
}

/**
 * @brief Read temperature from the sensor
 * 
 * @return float Temperature in Celsius
 */
float mcp9700_read_temperature(void) {
    uint16_t raw_value = mcp9700_read_raw();
    
    // Convert ADC reading to voltage (in millivolts)
    float voltage_mv = raw_value * adc_voltage_scale * 1000.0f;
    
    // Calculate temperature using the sensor's transfer function
    // T(°C) = (VOUT - V0°C) / TC
    float temperature = (voltage_mv - MCP9700_V0C_MV) / MCP9700_TC_MV;
    
    return temperature;
}

/**
 * @brief Get the ADC channel used by the sensor
 * 
 * @return uint8_t ADC channel
 */
uint8_t mcp9700_get_adc_channel(void) {
    return adc_channel;
}
