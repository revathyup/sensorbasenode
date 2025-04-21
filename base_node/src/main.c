/**
 * @file main.c
 * @brief Main application for base node
 */
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/sys/printk.h>
#include <zephyr/console/console.h>
#include <zephyr/shell/shell.h>
#include <zephyr/logging/log.h>
#include <string.h>

#include "sensor_driver.h"

LOG_MODULE_REGISTER(main, CONFIG_LOG_DEFAULT_LEVEL);

// Device references
static const struct device *sensor_node_dev;

// Forward declarations
static void trigger_handler(const struct device *dev, const struct sensor_trigger *trigger);

/**
 * @brief Command handler for setting light threshold
 */
static int cmd_light_threshold(const struct shell *shell, size_t argc, char **argv) {
    if (argc < 3) {
        shell_error(shell, "Usage: light_threshold <type> <value>");
        shell_info(shell, "  type: 'min' or 'max'");
        shell_info(shell, "  value: threshold value (0-65535)");
        return -EINVAL;
    }
    
    uint8_t threshold_type;
    if (strcmp(argv[1], "min") == 0) {
        threshold_type = THRESHOLD_TYPE_MIN;
    } else if (strcmp(argv[1], "max") == 0) {
        threshold_type = THRESHOLD_TYPE_MAX;
    } else {
        shell_error(shell, "Invalid threshold type. Use 'min' or 'max'");
        return -EINVAL;
    }
    
    uint16_t threshold = (uint16_t)strtoul(argv[2], NULL, 10);
    
    int ret = sensor_node_set_light_threshold(sensor_node_dev, threshold, threshold_type);
    if (ret < 0) {
        shell_error(shell, "Failed to set light threshold: %d", ret);
        return ret;
    }
    
    shell_print(shell, "Light threshold set to %u (type: %s)", 
               threshold, threshold_type == THRESHOLD_TYPE_MIN ? "min" : "max");
    return 0;
}

/**
 * @brief Command handler for setting temperature threshold
 */
static int cmd_temp_threshold(const struct shell *shell, size_t argc, char **argv) {
    if (argc < 3) {
        shell_error(shell, "Usage: temp_threshold <type> <value>");
        shell_info(shell, "  type: 'min' or 'max'");
        shell_info(shell, "  value: temperature in degrees C (can be decimal, e.g. 25.5)");
        return -EINVAL;
    }
    
    uint8_t threshold_type;
    if (strcmp(argv[1], "min") == 0) {
        threshold_type = THRESHOLD_TYPE_MIN;
    } else if (strcmp(argv[1], "max") == 0) {
        threshold_type = THRESHOLD_TYPE_MAX;
    } else {
        shell_error(shell, "Invalid threshold type. Use 'min' or 'max'");
        return -EINVAL;
    }
    
    float temp = strtof(argv[2], NULL);
    uint16_t threshold = (uint16_t)(temp * 10); // Convert to fixed-point (0.1°C resolution)
    
    int ret = sensor_node_set_temperature_threshold(sensor_node_dev, threshold, threshold_type);
    if (ret < 0) {
        shell_error(shell, "Failed to set temperature threshold: %d", ret);
        return ret;
    }
    
    shell_print(shell, "Temperature threshold set to %.1f°C (type: %s)", 
               temp, threshold_type == THRESHOLD_TYPE_MIN ? "min" : "max");
    return 0;
}

/**
 * @brief Command handler for setting moisture threshold
 */
static int cmd_moisture_threshold(const struct shell *shell, size_t argc, char **argv) {
    if (argc < 3) {
        shell_error(shell, "Usage: moisture_threshold <type> <value>");
        shell_info(shell, "  type: 'min' or 'max'");
        shell_info(shell, "  value: threshold value (0-1023)");
        return -EINVAL;
    }
    
    uint8_t threshold_type;
    if (strcmp(argv[1], "min") == 0) {
        threshold_type = THRESHOLD_TYPE_MIN;
    } else if (strcmp(argv[1], "max") == 0) {
        threshold_type = THRESHOLD_TYPE_MAX;
    } else {
        shell_error(shell, "Invalid threshold type. Use 'min' or 'max'");
        return -EINVAL;
    }
    
    uint16_t threshold = (uint16_t)strtoul(argv[2], NULL, 10);
    
    int ret = sensor_node_set_moisture_threshold(sensor_node_dev, threshold, threshold_type);
    if (ret < 0) {
        shell_error(shell, "Failed to set moisture threshold: %d", ret);
        return ret;
    }
    
    shell_print(shell, "Moisture threshold set to %u (type: %s)", 
               threshold, threshold_type == THRESHOLD_TYPE_MIN ? "min" : "max");
    return 0;
}

/**
 * @brief Command handler for reading sensor values
 */
static int cmd_read_sensors(const struct shell *shell, size_t argc, char **argv) {
    struct sensor_value light_val, temp_val, moisture_val;
    int ret;
    
    // Fetch sensor data
    ret = sensor_sample_fetch(sensor_node_dev);
    if (ret < 0) {
        shell_error(shell, "Failed to fetch sensor data: %d", ret);
        return ret;
    }
    
    // Get sensor values
    ret = sensor_channel_get(sensor_node_dev, SENSOR_NODE_CHAN_LIGHT, &light_val);
    if (ret < 0) {
        shell_error(shell, "Failed to get light sensor value: %d", ret);
        return ret;
    }
    
    ret = sensor_channel_get(sensor_node_dev, SENSOR_NODE_CHAN_TEMPERATURE, &temp_val);
    if (ret < 0) {
        shell_error(shell, "Failed to get temperature sensor value: %d", ret);
        return ret;
    }
    
    ret = sensor_channel_get(sensor_node_dev, SENSOR_NODE_CHAN_SOIL_MOISTURE, &moisture_val);
    if (ret < 0) {
        shell_error(shell, "Failed to get moisture sensor value: %d", ret);
        return ret;
    }
    
    // Print sensor values
    shell_print(shell, "Sensor readings:");
    shell_print(shell, "  Light: %d", light_val.val1);
    shell_print(shell, "  Temperature: %.1f°C", sensor_value_to_double(&temp_val));
    shell_print(shell, "  Soil Moisture: %d", moisture_val.val1);
    
    return 0;
}

/**
 * @brief Command handler for resetting sensor thresholds
 */
static int cmd_reset_thresholds(const struct shell *shell, size_t argc, char **argv) {
    int ret = sensor_node_send_command(sensor_node_dev, CMD_RESET, NULL, 0);
    if (ret < 0) {
        shell_error(shell, "Failed to reset thresholds: %d", ret);
        return ret;
    }
    
    shell_print(shell, "Sensor thresholds reset to default values");
    return 0;
}

// Define shell commands
SHELL_STATIC_SUBCMD_SET_CREATE(sub_sensors,
    SHELL_CMD(read, NULL, "Read all sensor values", cmd_read_sensors),
    SHELL_CMD(light_threshold, NULL, "Set light threshold (min/max value)", cmd_light_threshold),
    SHELL_CMD(temp_threshold, NULL, "Set temperature threshold (min/max value)", cmd_temp_threshold),
    SHELL_CMD(moisture_threshold, NULL, "Set moisture threshold (min/max value)", cmd_moisture_threshold),
    SHELL_CMD(reset, NULL, "Reset all thresholds to default values", cmd_reset_thresholds),
    SHELL_SUBCMD_SET_END
);

SHELL_CMD_REGISTER(sensors, &sub_sensors, "Sensor commands", NULL);

/**
 * @brief Sensor trigger handler
 */
static void trigger_handler(const struct device *dev, const struct sensor_trigger *trigger) {
    struct sensor_value val;
    
    switch (trigger->chan) {
        case SENSOR_NODE_CHAN_LIGHT:
            sensor_channel_get(dev, SENSOR_NODE_CHAN_LIGHT, &val);
            printk("Light trigger: value=%d\n", val.val1);
            break;
            
        case SENSOR_NODE_CHAN_TEMPERATURE: {
            sensor_channel_get(dev, SENSOR_NODE_CHAN_TEMPERATURE, &val);
            double temp = sensor_value_to_double(&val);
            printk("Temperature trigger: value=%.1f°C\n", temp);
            break;
        }
        
        case SENSOR_NODE_CHAN_SOIL_MOISTURE:
            sensor_channel_get(dev, SENSOR_NODE_CHAN_SOIL_MOISTURE, &val);
            printk("Soil moisture trigger: value=%d\n", val.val1);
            break;
            
        default:
            printk("Unknown trigger channel\n");
            break;
    }
}

/**
 * @brief Set up triggers for all sensors
 */
static void setup_triggers(void) {
    struct sensor_trigger trig;
    
    // Set up light sensor trigger
    trig.type = SENSOR_TRIG_THRESHOLD;
    trig.chan = SENSOR_NODE_CHAN_LIGHT;
    sensor_trigger_set(sensor_node_dev, &trig, trigger_handler);
    
    // Set up temperature sensor trigger
    trig.type = SENSOR_TRIG_THRESHOLD;
    trig.chan = SENSOR_NODE_CHAN_TEMPERATURE;
    sensor_trigger_set(sensor_node_dev, &trig, trigger_handler);
    
    // Set up moisture sensor trigger
    trig.type = SENSOR_TRIG_THRESHOLD;
    trig.chan = SENSOR_NODE_CHAN_SOIL_MOISTURE;
    sensor_trigger_set(sensor_node_dev, &trig, trigger_handler);
}

/**
 * @brief Main application entry point
 */
int main(void) {
    printk("Smart Gardening System - Base Node\n");
    
    // Initialize console
    console_init();
    
    // Get device reference
    sensor_node_dev = DEVICE_DT_GET_ONE(sensor_node);
    
    if (!device_is_ready(sensor_node_dev)) {
        printk("Sensor node device not ready\n");
        return -1;
    }
    
    printk("Sensor node device initialized\n");
    
    // Set up triggers
    setup_triggers();
    
    // Continuous monitoring loop
    while (1) {
        // Fetch and print sensor data
        if (sensor_sample_fetch(sensor_node_dev) == 0) {
            struct sensor_value light_val, temp_val, moisture_val;
            
            sensor_channel_get(sensor_node_dev, SENSOR_NODE_CHAN_LIGHT, &light_val);
            sensor_channel_get(sensor_node_dev, SENSOR_NODE_CHAN_TEMPERATURE, &temp_val);
            sensor_channel_get(sensor_node_dev, SENSOR_NODE_CHAN_SOIL_MOISTURE, &moisture_val);
            
            printk("Light: %d, Temp: %.1f°C, Moisture: %d\n",
                   light_val.val1, sensor_value_to_double(&temp_val), moisture_val.val1);
        }
        
        // Sleep for 5 seconds
        k_sleep(K_SECONDS(5));
    }
    
    return 0;
}
