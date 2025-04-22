/**
 * @file main.c
 * @brief Minimal Smart Gardening System sensor node
 */

#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "hardware/adc.h"
#include "hardware/gpio.h"

// Pin definitions
#define TEMP_SENSOR_PIN 26 // ADC0

// MCP9700 has a 10mV/°C output with a 500mV offset at 0°C
#define MCP9700_TEMP_COEF_MV 10.0f
#define MCP9700_OFFSET_MV    500.0f

/**
 * @brief Initialize the MCP9700 temperature sensor
 * 
 * @param pin ADC pin number
 * @return true if initialization successful, false otherwise
 */
bool mcp9700_init(uint pin) {
    // Initialize ADC
    adc_init();
    
    // Configure ADC pin
    adc_gpio_init(pin);
    
    // Select ADC input (0-3 are available)
    adc_select_input(0); // ADC0 corresponds to GPIO26
    
    return true;
}

/**
 * @brief Read temperature from the MCP9700 sensor
 * 
 * @return float Temperature in Celsius
 */
float mcp9700_read_temperature(void) {
    // Read ADC value (12-bit conversion, 0-4095)
    uint16_t adc_raw = adc_read();
    
    // Calculate voltage (in mV)
    // Pico's ADC reference voltage is 3.3V (3300mV)
    float voltage_mv = (float)adc_raw * 3300.0f / 4096.0f;
    
    // Convert voltage to temperature (MCP9700 outputs 10mV/°C with 500mV at 0°C)
    float temperature = (voltage_mv - MCP9700_OFFSET_MV) / MCP9700_TEMP_COEF_MV;
    
    return temperature;
}

/**
 * @brief Main application entry point
 */
int main() {
    // Initialize standard I/O over USB
    stdio_init_all();
    
    // Wait for USB connection to be established
    sleep_ms(2000);
    
    printf("Smart Gardening System - Minimal Sensor Node starting...\n");
    
    // Initialize temperature sensor
    printf("Initializing temperature sensor...\n");
    if (!mcp9700_init(TEMP_SENSOR_PIN)) {
        printf("Failed to initialize MCP9700 temperature sensor!\n");
        return -1;
    }
    
    printf("MCP9700 temperature sensor initialized successfully\n");
    printf("Sensor node ready!\n");
    
    while (1) {
        // Read temperature sensor
        float temperature = mcp9700_read_temperature();
        
        // Convert to formatted value for display
        int temp_whole = (int)temperature;
        int temp_frac = (int)(temperature * 100) % 100;
        
        // Display temperature
        printf("Temperature: %d.%02d°C\n", temp_whole, temp_frac);
        
        // Small delay between readings
        sleep_ms(2000);
    }
    
    return 0;
}