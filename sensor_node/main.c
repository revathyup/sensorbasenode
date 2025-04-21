/**
 * @file main.c
 * @brief Main file for the sensor node application
 */

#include <stdio.h>
#include <stdlib.h>
#include "pico/stdlib.h"
#include "pico/time.h"
#include "sensors.h"
#include "uart_protocol.h"

// Constants
#define LED_PIN 25  // Built-in LED on Pico

// Function declarations
static void init_system(void);
static void process_sensors(void);

// Main function
int main() {
    // Initialize system
    init_system();
    
    // Main loop
    while (1) {
        // Process any incoming UART messages
        uart_protocol_process_messages();
        
        // Read and process sensor data
        process_sensors();
        
        // Get current configuration for sampling rate
        sensor_config_t config;
        sensors_get_config(&config);
        
        // Sleep until next sample time
        sleep_ms(config.sampling_rate_ms);
    }
    
    return 0;
}

/**
 * @brief Initialize system components
 */
static void init_system(void) {
    // Initialize stdio
    stdio_init_all();
    sleep_ms(2000);  // Give USB time to initialize
    
    printf("\n\n--- Sensor Node Starting ---\n");
    
    // Initialize GPIO
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
    
    // Initialize sensors
    if (!sensors_init()) {
        printf("ERROR: Failed to initialize all sensors\n");
    }
    
    // Initialize UART protocol
    if (!uart_protocol_init()) {
        printf("ERROR: Failed to initialize UART protocol\n");
    }
    
    printf("System initialization complete\n");
    
    // Blink LED to indicate successful initialization
    for (int i = 0; i < 3; i++) {
        gpio_put(LED_PIN, 1);
        sleep_ms(100);
        gpio_put(LED_PIN, 0);
        sleep_ms(100);
    }
}

/**
 * @brief Read and process sensor data
 */
static void process_sensors(void) {
    sensor_data_t data;
    interrupt_status_t int_status;
    
    // Read sensor data
    if (sensors_read(&data)) {
        // Print sensor data to console for debugging
        printf("Sensor Readings: ");
        if (data.valid_light) {
            printf("Light: %.2f lux, ", data.light);
        } else {
            printf("Light: INVALID, ");
        }
        
        if (data.valid_temperature) {
            printf("Temp: %.2f°C, ", data.temperature);
        } else {
            printf("Temp: INVALID, ");
        }
        
        if (data.valid_soil) {
            printf("Soil: %.2f%%", data.soil_moisture);
        } else {
            printf("Soil: INVALID");
        }
        printf("\n");
        
        // Send sensor data over UART
        uart_protocol_send_sensor_data(&data);
        
        // Check for interrupt conditions
        if (sensors_check_interrupts(&data, &int_status)) {
            // Print interrupt status for debugging
            printf("INTERRUPT: ");
            if (int_status.temp_high) printf("High Temperature, ");
            if (int_status.soil_low) printf("Low Soil Moisture, ");
            if (int_status.light_low) printf("Low Light, ");
            if (int_status.light_high) printf("High Light");
            printf("\n");
            
            // Send interrupt notification
            uart_protocol_send_interrupt(&data, &int_status);
            
            // Blink LED rapidly to indicate interrupt
            for (int i = 0; i < 5; i++) {
                gpio_put(LED_PIN, 1);
                sleep_ms(50);
                gpio_put(LED_PIN, 0);
                sleep_ms(50);
            }
        } else {
            // Toggle LED to indicate normal operation
            gpio_put(LED_PIN, !gpio_get(LED_PIN));
        }
    } else {
        // Error reading sensors
        printf("ERROR: Failed to read any sensor data\n");
        uart_protocol_send_error(ERR_SENSOR_FAIL, 0);
    }
}
