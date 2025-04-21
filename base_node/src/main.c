/**
 * @file main.c
 * @brief Main application for the base node
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/printk.h>
#include <zephyr/shell/shell.h>
#include <stdlib.h>

#include "protocol.h"
#include "sensor_node_driver.h"

// LED gpio
#define LED_NODE        DT_ALIAS(led0)
#define LED             DT_GPIO_LABEL(LED_NODE, gpios)
#define LED_PIN         DT_GPIO_PIN(LED_NODE, gpios)
#define LED_FLAGS       DT_GPIO_FLAGS(LED_NODE, gpios)

// Constants
#define SENSOR_READING_INTERVAL_MS 1000

// Function declarations
static void process_data(void);
static void process_interrupts(void);
static void print_sensor_value(const struct sensor_value *val);
static int cmd_set_threshold(const struct shell *shell, size_t argc, char **argv);
static int cmd_get_readings(const struct shell *shell, size_t argc, char **argv);
static int cmd_set_sampling_rate(const struct shell *shell, size_t argc, char **argv);

// LED device
static const struct device *led_dev;

// Shell commands
SHELL_STATIC_SUBCMD_SET_CREATE(threshold_cmds,
    SHELL_CMD(temp_high, NULL, "Set high temperature threshold (°C)\n"
              "Usage: set_threshold temp_high <value>", cmd_set_threshold),
    SHELL_CMD(soil_low, NULL, "Set low soil moisture threshold (%)\n"
              "Usage: set_threshold soil_low <value>", cmd_set_threshold),
    SHELL_CMD(light_low, NULL, "Set low light threshold (lux)\n"
              "Usage: set_threshold light_low <value>", cmd_set_threshold),
    SHELL_CMD(light_high, NULL, "Set high light threshold (lux)\n"
              "Usage: set_threshold light_high <value>", cmd_set_threshold),
    SHELL_SUBCMD_SET_END
);

SHELL_STATIC_SUBCMD_SET_CREATE(sub_commands,
    SHELL_CMD(set_threshold, &threshold_cmds, "Set sensor thresholds", NULL),
    SHELL_CMD(get_readings, NULL, "Get current sensor readings", cmd_get_readings),
    SHELL_CMD(set_sampling_rate, NULL, "Set sampling rate (in Hz)\n"
              "Usage: set_sampling_rate <value>", cmd_set_sampling_rate),
    SHELL_SUBCMD_SET_END
);

SHELL_CMD_REGISTER(sensor, &sub_commands, "Sensor node commands", NULL);

// Main function
void main(void)
{
    int ret;
    
    printk("Sensor Node Base - Zephyr Driver Demo\n");
    
    // Initialize LED
    led_dev = device_get_binding(LED);
    if (led_dev == NULL) {
        printk("Error: Failed to get LED device\n");
        return;
    }
    
    ret = gpio_pin_configure(led_dev, LED_PIN, GPIO_OUTPUT_ACTIVE | LED_FLAGS);
    if (ret < 0) {
        printk("Error: Failed to configure LED pin: %d\n", ret);
        return;
    }
    
    // Verify sensor node driver is available
    const struct device *sensor_dev = sensor_node_driver_get_device();
    if (sensor_dev == NULL) {
        printk("Error: Failed to get sensor node device\n");
        return;
    }
    
    // Ensure sensor is ready
    if (!device_is_ready(sensor_dev)) {
        printk("Error: Sensor node device not ready\n");
        return;
    }
    
    printk("Sensor node driver initialized successfully\n");
    
    // Main loop
    while (1) {
        // Process sensor data
        process_data();
        
        // Process any interrupts
        process_interrupts();
        
        // Sleep for interval
        k_sleep(K_MSEC(SENSOR_READING_INTERVAL_MS));
    }
}

/**
 * @brief Process sensor data
 */
static void process_data(void)
{
    const struct device *sensor_dev = sensor_node_driver_get_device();
    if (sensor_dev == NULL) {
        return;
    }
    
    // Fetch sensor samples
    int ret = sensor_sample_fetch(sensor_dev);
    if (ret < 0) {
        printk("Error: Failed to fetch sensor samples: %d\n", ret);
        return;
    }
    
    // Read temperature
    struct sensor_value temp_val;
    ret = sensor_channel_get(sensor_dev, SENSOR_CHAN_AMBIENT_TEMP, &temp_val);
    if (ret == 0) {
        printk("Temperature: ");
        print_sensor_value(&temp_val);
        printk(" °C\n");
    }
    
    // Read soil moisture
    struct sensor_value soil_val;
    ret = sensor_channel_get(sensor_dev, SENSOR_CHAN_HUMIDITY, &soil_val);
    if (ret == 0) {
        printk("Soil Moisture: ");
        print_sensor_value(&soil_val);
        printk(" %%\n");
    }
    
    // Read light level
    struct sensor_value light_val;
    ret = sensor_channel_get(sensor_dev, SENSOR_CHAN_LIGHT, &light_val);
    if (ret == 0) {
        printk("Light: ");
        print_sensor_value(&light_val);
        printk(" lux\n");
    }
    
    printk("\n");
}

/**
 * @brief Process interrupts
 */
static void process_interrupts(void)
{
    const struct device *sensor_dev = sensor_node_driver_get_device();
    if (sensor_dev == NULL) {
        return;
    }
    
    // Check each interrupt type
    if (sensor_node_interrupt_is_triggered(sensor_dev, INT_HIGH_TEMPERATURE)) {
        printk("*** ALERT: HIGH TEMPERATURE DETECTED ***\n");
        
        // Blink LED rapidly 5 times
        for (int i = 0; i < 5; i++) {
            gpio_pin_set(led_dev, LED_PIN, 1);
            k_sleep(K_MSEC(100));
            gpio_pin_set(led_dev, LED_PIN, 0);
            k_sleep(K_MSEC(100));
        }
        
        // Clear the interrupt
        sensor_node_clear_interrupt(sensor_dev, INT_HIGH_TEMPERATURE);
    }
    
    if (sensor_node_interrupt_is_triggered(sensor_dev, INT_LOW_SOIL_MOISTURE)) {
        printk("*** ALERT: LOW SOIL MOISTURE DETECTED ***\n");
        
        // Clear the interrupt
        sensor_node_clear_interrupt(sensor_dev, INT_LOW_SOIL_MOISTURE);
    }
    
    if (sensor_node_interrupt_is_triggered(sensor_dev, INT_LOW_LIGHT)) {
        printk("*** ALERT: LOW LIGHT DETECTED ***\n");
        
        // Clear the interrupt
        sensor_node_clear_interrupt(sensor_dev, INT_LOW_LIGHT);
    }
    
    if (sensor_node_interrupt_is_triggered(sensor_dev, INT_HIGH_LIGHT)) {
        printk("*** ALERT: HIGH LIGHT DETECTED ***\n");
        
        // Clear the interrupt
        sensor_node_clear_interrupt(sensor_dev, INT_HIGH_LIGHT);
    }
    
    // Check for errors
    uint8_t error = sensor_node_get_last_error(sensor_dev);
    if (error != 0) {
        printk("Sensor node error: %d\n", error);
    }
}

/**
 * @brief Print a sensor value
 */
static void print_sensor_value(const struct sensor_value *val)
{
    printk("%d.%06d", val->val1, val->val2);
}

/**
 * @brief Shell command to set a threshold
 */
static int cmd_set_threshold(const struct shell *shell, size_t argc, char **argv)
{
    if (argc != 2) {
        shell_error(shell, "Wrong number of arguments");
        return -EINVAL;
    }
    
    // Parse value
    char *endptr;
    float value = strtof(argv[1], &endptr);
    if (*endptr != '\0') {
        shell_error(shell, "Invalid number: %s", argv[1]);
        return -EINVAL;
    }
    
    // Get device
    const struct device *sensor_dev = sensor_node_driver_get_device();
    if (sensor_dev == NULL) {
        shell_error(shell, "Sensor device not available");
        return -ENODEV;
    }
    
    // Determine which threshold to set
    enum config_param param;
    if (strcmp(argv[-1], "temp_high") == 0) {
        param = CONFIG_TEMP_HIGH_THRESHOLD;
        shell_print(shell, "Setting high temperature threshold to %.2f °C", value);
    }
    else if (strcmp(argv[-1], "soil_low") == 0) {
        param = CONFIG_SOIL_LOW_THRESHOLD;
        shell_print(shell, "Setting low soil moisture threshold to %.2f %%", value);
    }
    else if (strcmp(argv[-1], "light_low") == 0) {
        param = CONFIG_LIGHT_LOW_THRESHOLD;
        shell_print(shell, "Setting low light threshold to %.2f lux", value);
    }
    else if (strcmp(argv[-1], "light_high") == 0) {
        param = CONFIG_LIGHT_HIGH_THRESHOLD;
        shell_print(shell, "Setting high light threshold to %.2f lux", value);
    }
    else {
        shell_error(shell, "Unknown threshold type");
        return -EINVAL;
    }
    
    // Set the threshold
    int ret = sensor_node_set_config(sensor_dev, param, (int32_t)(value * 100.0f));
    if (ret != 0) {
        shell_error(shell, "Failed to set threshold: %d", ret);
        return ret;
    }
    
    shell_print(shell, "Threshold set successfully");
    return 0;
}

/**
 * @brief Shell command to get sensor readings
 */
static int cmd_get_readings(const struct shell *shell, size_t argc, char **argv)
{
    const struct device *sensor_dev = sensor_node_driver_get_device();
    if (sensor_dev == NULL) {
        shell_error(shell, "Sensor device not available");
        return -ENODEV;
    }
    
    // Fetch sensor samples
    int ret = sensor_sample_fetch(sensor_dev);
    if (ret < 0) {
        shell_error(shell, "Failed to fetch sensor samples: %d", ret);
        return ret;
    }
    
    // Read temperature
    struct sensor_value temp_val;
    ret = sensor_channel_get(sensor_dev, SENSOR_CHAN_AMBIENT_TEMP, &temp_val);
    if (ret == 0) {
        shell_print(shell, "Temperature: %d.%06d °C", temp_val.val1, temp_val.val2);
    } else {
        shell_print(shell, "Temperature: Not available");
    }
    
    // Read soil moisture
    struct sensor_value soil_val;
    ret = sensor_channel_get(sensor_dev, SENSOR_CHAN_HUMIDITY, &soil_val);
    if (ret == 0) {
        shell_print(shell, "Soil Moisture: %d.%06d %%", soil_val.val1, soil_val.val2);
    } else {
        shell_print(shell, "Soil Moisture: Not available");
    }
    
    // Read light level
    struct sensor_value light_val;
    ret = sensor_channel_get(sensor_dev, SENSOR_CHAN_LIGHT, &light_val);
    if (ret == 0) {
        shell_print(shell, "Light: %d.%06d lux", light_val.val1, light_val.val2);
    } else {
        shell_print(shell, "Light: Not available");
    }
    
    return 0;
}

/**
 * @brief Shell command to set sampling rate
 */
static int cmd_set_sampling_rate(const struct shell *shell, size_t argc, char **argv)
{
    if (argc != 2) {
        shell_error(shell, "Wrong number of arguments");
        return -EINVAL;
    }
    
    // Parse value
    char *endptr;
    float value = strtof(argv[1], &endptr);
    if (*endptr != '\0' || value <= 0) {
        shell_error(shell, "Invalid sampling rate: %s", argv[1]);
        return -EINVAL;
    }
    
    // Get device
    const struct device *sensor_dev = sensor_node_driver_get_device();
    if (sensor_dev == NULL) {
        shell_error(shell, "Sensor device not available");
        return -ENODEV;
    }
    
    // Convert to sampling rate in ms
    int32_t sampling_rate_ms = (int32_t)(1000.0f / value);
    
    shell_print(shell, "Setting sampling rate to %.2f Hz (interval: %d ms)",
               value, sampling_rate_ms);
    
    // Set the sampling rate
    int ret = sensor_node_set_config(sensor_dev, CONFIG_SAMPLING_RATE, sampling_rate_ms);
    if (ret != 0) {
        shell_error(shell, "Failed to set sampling rate: %d", ret);
        return ret;
    }
    
    shell_print(shell, "Sampling rate set successfully");
    return 0;
}
