/**
 * @file main.c
 * @brief Simplified version of main application for sensor node using Zephyr RTOS
 *        Only focusing on the temperature sensor for now
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/console/console.h>
#include <zephyr/logging/log.h>

#include "mcp9700.h"

LOG_MODULE_REGISTER(main, CONFIG_LOG_DEFAULT_LEVEL);

/* Define device tree labels for peripherals */
#define ADC_DEV_NODE        DT_ALIAS(adc0)
#define UART_DEV_NODE       DT_ALIAS(uart0)

/* Temperature sensor ADC channel */
#define TEMP_ADC_CHANNEL    0

/* Sensor sampling periods in milliseconds */
#define TEMP_SAMPLING_PERIOD_MS       2000

/* Device handles */
static const struct device *adc_dev;
static const struct device *uart_dev;

/**
 * @brief Main function
 */
int main(void) {
    int ret;
    float temperature;
    int32_t scaled_temp;
    
    LOG_INF("Smart Gardening System - Simplified Sensor Node starting...");
    printk("Smart Gardening System - Simplified Sensor Node\n");
    
    // Initialize console for output
    console_init();
    
    // Get device handles from device tree
    adc_dev = DEVICE_DT_GET(ADC_DEV_NODE);
    
    // Initialize peripherals
    if (!device_is_ready(adc_dev)) {
        LOG_ERR("ADC device not ready");
        printk("ERROR: ADC device not ready\n");
        return -1;
    }
    
    // Initialize temperature sensor
    if (!mcp9700_init(adc_dev, TEMP_ADC_CHANNEL)) {
        LOG_ERR("Failed to initialize MCP9700 temperature sensor");
        printk("ERROR: Failed to initialize temperature sensor\n");
        return -1;
    }
    
    LOG_INF("Temperature sensor initialized successfully");
    printk("Temperature sensor initialized successfully\n");
    
    // Main loop - read temperature and print to console
    while (1) {
        // Read temperature sensor
        temperature = mcp9700_read_temperature();
        scaled_temp = (int32_t)(temperature * 100);
        
        LOG_INF("Temperature: %d.%02d°C", scaled_temp / 100, scaled_temp % 100);
        printk("Temperature: %d.%02d°C\n", scaled_temp / 100, scaled_temp % 100);
        
        // Wait for next sampling period
        k_msleep(TEMP_SAMPLING_PERIOD_MS);
    }
    
    return 0;
}