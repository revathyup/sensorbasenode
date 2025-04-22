/**
 * @file main.c
 * @brief Main application for light sensor node
 */
#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "hardware/i2c.h"

#include "tsl2591.h"

// Pin definitions for I2C
#define I2C_SDA_PIN 4
#define I2C_SCL_PIN 5

// Light sensor reading interval (ms)
#define LIGHT_READING_INTERVAL_MS 2000

// Light threshold for alerts (in lux)
#define LIGHT_LOW_THRESHOLD 20.0f
#define LIGHT_HIGH_THRESHOLD 1000.0f

/**
 * @brief Main application entry point
 */
int main() {
    // Initialize standard I/O over USB
    stdio_init_all();
    
    // Wait for USB connection to be established
    sleep_ms(2000);
    
    printf("Light Sensor Node starting...\n");
    
    // Initialize I2C for light sensor
    i2c_init(i2c0, 100 * 1000);  // 100 kHz
    gpio_set_function(I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_PIN);
    gpio_pull_up(I2C_SCL_PIN);
    bi_decl(bi_2pins_with_func(I2C_SDA_PIN, I2C_SCL_PIN, GPIO_FUNC_I2C));
    
    // Initialize light sensor
    printf("Initializing TSL2591 light sensor...\n");
    if (!tsl2591_init(i2c0)) {
        printf("Failed to initialize TSL2591 light sensor!\n");
        return -1;
    }
    
    // Configure sensor settings
    tsl2591_set_gain(TSL2591_GAIN_MED);
    tsl2591_set_integration_time(TSL2591_INTEGRATIONTIME_300);
    
    printf("Light sensor node ready!\n");
    
    while (1) {
        // Read light sensor values
        uint16_t full, ir;
        if (tsl2591_get_full_luminosity(&full, &ir)) {
            // Calculate lux value
            float lux = tsl2591_calculate_lux(full, ir);
            
            // Get visible light
            uint16_t visible = full - ir;
            
            // Display readings
            printf("Light readings: Full: %u, IR: %u, Visible: %u, Lux: %.2f\n",
                   full, ir, visible, lux);
            
            // Check thresholds
            if (lux < LIGHT_LOW_THRESHOLD) {
                printf("ALERT: Light level below threshold (%.2f lux < %.2f lux)\n",
                       lux, LIGHT_LOW_THRESHOLD);
            } else if (lux > LIGHT_HIGH_THRESHOLD) {
                printf("ALERT: Light level above threshold (%.2f lux > %.2f lux)\n", 
                       lux, LIGHT_HIGH_THRESHOLD);
            }
        } else {
            printf("Failed to read from light sensor\n");
        }
        
        // Wait before next reading
        sleep_ms(LIGHT_READING_INTERVAL_MS);
    }
    
    return 0;
}