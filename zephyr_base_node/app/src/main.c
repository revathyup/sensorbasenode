/**
 * @file main.c
 * @brief Main application for the Smart Gardening System base node
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/shell/shell.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/printk.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include "../../protocol.h"

LOG_MODULE_REGISTER(main, CONFIG_LOG_DEFAULT_LEVEL);

/* Device reference for the sensor node */
static const struct device *sensor_node_dev;

/* Threshold values */
static float light_low_threshold = 50.0f;    /* 50 lux */
static float light_high_threshold = 10000.0f; /* 10,000 lux */
static float temp_low_threshold = 10.0f;     /* 10°C */
static float temp_high_threshold = 30.0f;    /* 30°C */
static uint16_t soil_dry_threshold = 300;    /* 300 (30%) */
static uint16_t soil_wet_threshold = 700;    /* 700 (70%) */

/* Thread for periodic sensor reading */
#define SENSOR_THREAD_STACK_SIZE 2048
#define SENSOR_THREAD_PRIORITY 5

K_THREAD_STACK_DEFINE(sensor_thread_stack, SENSOR_THREAD_STACK_SIZE);
static struct k_thread sensor_thread_data;
static k_tid_t sensor_thread_id;
static volatile bool sensor_thread_running = false;

/* Forward declarations */
static int cmd_help(const struct shell *shell, size_t argc, char **argv);
static int cmd_ping(const struct shell *shell, size_t argc, char **argv);
static int cmd_show(const struct shell *shell, size_t argc, char **argv);
static int cmd_set(const struct shell *shell, size_t argc, char **argv);

/* Shell command handlers */
SHELL_STATIC_SUBCMD_SET_CREATE(sub_set,
    SHELL_CMD(light, NULL, "Set light threshold: set light low|high <value>", cmd_set),
    SHELL_CMD(temp, NULL, "Set temperature threshold: set temp low|high <value>", cmd_set),
    SHELL_CMD(soil, NULL, "Set soil moisture threshold: set soil dry|wet <value>", cmd_set),
    SHELL_SUBCMD_SET_END
);

SHELL_STATIC_SUBCMD_SET_CREATE(sub_show,
    SHELL_CMD(light, NULL, "Show light sensor data", cmd_show),
    SHELL_CMD(temp, NULL, "Show temperature sensor data", cmd_show),
    SHELL_CMD(soil, NULL, "Show soil moisture sensor data", cmd_show),
    SHELL_CMD(all, NULL, "Show all sensor data", cmd_show),
    SHELL_CMD(thresholds, NULL, "Show all threshold values", cmd_show),
    SHELL_SUBCMD_SET_END
);

SHELL_STATIC_SUBCMD_SET_CREATE(sub_commands,
    SHELL_CMD(help, NULL, "Display help information", cmd_help),
    SHELL_CMD(ping, NULL, "Ping the sensor node", cmd_ping),
    SHELL_CMD(show, &sub_show, "Show sensor data", NULL),
    SHELL_CMD(set, &sub_set, "Set sensor thresholds", NULL),
    SHELL_SUBCMD_SET_END
);

SHELL_CMD_REGISTER(help, &sub_commands, "Smart Gardening System Commands", cmd_help);

/* Shell command implementations */
static int cmd_help(const struct shell *shell, size_t argc, char **argv)
{
    shell_print(shell, "Smart Gardening System - Base Node");
    shell_print(shell, "Available commands:");
    shell_print(shell, "  help          - Display this help message");
    shell_print(shell, "  ping          - Ping the sensor node");
    shell_print(shell, "  show light    - Show light sensor data");
    shell_print(shell, "  show temp     - Show temperature sensor data");
    shell_print(shell, "  show soil     - Show soil moisture sensor data");
    shell_print(shell, "  show all      - Show all sensor data");
    shell_print(shell, "  show thresholds - Show all threshold values");
    shell_print(shell, "  set light low|high <value> - Set light threshold (lux)");
    shell_print(shell, "  set temp low|high <value>  - Set temperature threshold (°C)");
    shell_print(shell, "  set soil dry|wet <value>   - Set soil moisture threshold (0-1000)");
    return 0;
}

static int cmd_ping(const struct shell *shell, size_t argc, char **argv)
{
    int ret;
    
    shell_print(shell, "Pinging sensor node...");
    
    /* Send ping to sensor node via driver */
    ret = sensor_node_ping(sensor_node_dev);
    if (ret != 0) {
        shell_error(shell, "Failed to send ping: %d", ret);
        return ret;
    }
    
    /* Wait for response (ideally would wait for ACK) */
    k_sleep(K_MSEC(100));
    
    shell_print(shell, "Ping sent successfully");
    return 0;
}

static int cmd_show(const struct shell *shell, size_t argc, char **argv)
{
    struct sensor_value val;
    int ret;
    
    if (argc < 2) {
        shell_error(shell, "Missing parameter: show [light|temp|soil|all|thresholds]");
        return -EINVAL;
    }
    
    /* Fetch fresh data */
    ret = sensor_sample_fetch(sensor_node_dev);
    if (ret != 0) {
        shell_error(shell, "Failed to fetch sensor data: %d", ret);
        return ret;
    }
    
    /* Wait for response */
    k_sleep(K_MSEC(500));
    
    /* Show the requested data */
    if (strcmp(argv[1], "light") == 0 || strcmp(argv[1], "all") == 0) {
        /* Get light data */
        ret = sensor_channel_get(sensor_node_dev, SENSOR_CHAN_LIGHT, &val);
        if (ret == 0) {
            shell_print(shell, "Light: %.2f lux", sensor_value_to_double(&val));
            
            sensor_channel_get(sensor_node_dev, SENSOR_CHAN_PROX, &val);
            shell_print(shell, "  Full spectrum: %d", val.val1);
            
            sensor_channel_get(sensor_node_dev, SENSOR_CHAN_IR, &val);
            shell_print(shell, "  Infrared: %d", val.val1);
            
            sensor_channel_get(sensor_node_dev, SENSOR_CHAN_BLUE, &val);
            shell_print(shell, "  Visible: %d", val.val1);
        } else {
            shell_error(shell, "Failed to get light data: %d", ret);
        }
    }
    
    if (strcmp(argv[1], "temp") == 0 || strcmp(argv[1], "all") == 0) {
        /* Get temperature data */
        ret = sensor_channel_get(sensor_node_dev, SENSOR_CHAN_AMBIENT_TEMP, &val);
        if (ret == 0) {
            shell_print(shell, "Temperature: %.2f °C", sensor_value_to_double(&val));
        } else {
            shell_error(shell, "Failed to get temperature data: %d", ret);
        }
    }
    
    if (strcmp(argv[1], "soil") == 0 || strcmp(argv[1], "all") == 0) {
        /* Get soil moisture data */
        ret = sensor_channel_get(sensor_node_dev, SENSOR_CHAN_HUMIDITY, &val);
        if (ret == 0) {
            shell_print(shell, "Soil moisture: %d (%.1f%%)", 
                       val.val1, (float)val.val2 / 10000.0f);
            
            /* Get soil temperature */
            ret = sensor_channel_get(sensor_node_dev, SENSOR_CHAN_DIE_TEMP, &val);
            if (ret == 0) {
                shell_print(shell, "Soil temperature: %.2f °C", sensor_value_to_double(&val));
            }
        } else {
            shell_error(shell, "Failed to get soil moisture data: %d", ret);
        }
    }
    
    if (strcmp(argv[1], "thresholds") == 0) {
        shell_print(shell, "Threshold values:");
        shell_print(shell, "  Light: %.2f lux (low) - %.2f lux (high)", 
                   light_low_threshold, light_high_threshold);
        shell_print(shell, "  Temperature: %.2f °C (low) - %.2f °C (high)", 
                   temp_low_threshold, temp_high_threshold);
        shell_print(shell, "  Soil moisture: %d (dry) - %d (wet)", 
                   soil_dry_threshold, soil_wet_threshold);
    }
    
    return 0;
}

static int cmd_set(const struct shell *shell, size_t argc, char **argv)
{
    if (argc < 4) {
        shell_error(shell, "Missing parameters: set [light|temp|soil] [low|high|dry|wet] <value>");
        return -EINVAL;
    }
    
    /* Extract value parameter */
    float value = strtof(argv[3], NULL);
    int ret = 0;
    
    /* Process based on sensor type */
    if (strcmp(argv[1], "light") == 0) {
        if (strcmp(argv[2], "low") == 0) {
            light_low_threshold = value;
            ret = sensor_node_set_threshold(sensor_node_dev, SENSOR_LIGHT, ALERT_LIGHT_LOW, value);
            shell_print(shell, "Set light low threshold to %.2f lux", value);
        } else if (strcmp(argv[2], "high") == 0) {
            light_high_threshold = value;
            ret = sensor_node_set_threshold(sensor_node_dev, SENSOR_LIGHT, ALERT_LIGHT_HIGH, value);
            shell_print(shell, "Set light high threshold to %.2f lux", value);
        } else {
            shell_error(shell, "Invalid parameter: %s (expected 'low' or 'high')", argv[2]);
            return -EINVAL;
        }
    } else if (strcmp(argv[1], "temp") == 0) {
        if (strcmp(argv[2], "low") == 0) {
            temp_low_threshold = value;
            ret = sensor_node_set_threshold(sensor_node_dev, SENSOR_TEMPERATURE, ALERT_TEMP_LOW, value);
            shell_print(shell, "Set temperature low threshold to %.2f °C", value);
        } else if (strcmp(argv[2], "high") == 0) {
            temp_high_threshold = value;
            ret = sensor_node_set_threshold(sensor_node_dev, SENSOR_TEMPERATURE, ALERT_TEMP_HIGH, value);
            shell_print(shell, "Set temperature high threshold to %.2f °C", value);
        } else {
            shell_error(shell, "Invalid parameter: %s (expected 'low' or 'high')", argv[2]);
            return -EINVAL;
        }
    } else if (strcmp(argv[1], "soil") == 0) {
        if (strcmp(argv[2], "dry") == 0) {
            soil_dry_threshold = (uint16_t)value;
            ret = sensor_node_set_threshold(sensor_node_dev, SENSOR_SOIL_MOISTURE, ALERT_SOIL_DRY, value);
            shell_print(shell, "Set soil dry threshold to %d", soil_dry_threshold);
        } else if (strcmp(argv[2], "wet") == 0) {
            soil_wet_threshold = (uint16_t)value;
            ret = sensor_node_set_threshold(sensor_node_dev, SENSOR_SOIL_MOISTURE, ALERT_SOIL_WET, value);
            shell_print(shell, "Set soil wet threshold to %d", soil_wet_threshold);
        } else {
            shell_error(shell, "Invalid parameter: %s (expected 'dry' or 'wet')", argv[2]);
            return -EINVAL;
        }
    } else {
        shell_error(shell, "Invalid parameter: %s (expected 'light', 'temp', or 'soil')", argv[1]);
        return -EINVAL;
    }
    
    if (ret != 0) {
        shell_error(shell, "Failed to set threshold: %d", ret);
    }
    
    return ret;
}

/* Sensor reading thread */
static void sensor_thread(void *arg1, void *arg2, void *arg3)
{
    struct sensor_value val;
    int ret;
    
    ARG_UNUSED(arg1);
    ARG_UNUSED(arg2);
    ARG_UNUSED(arg3);
    
    printk("Sensor reading thread started\n");
    
    while (sensor_thread_running) {
        /* Fetch and process sensor data every 5 seconds */
        ret = sensor_sample_fetch(sensor_node_dev);
        if (ret == 0) {
            /* Get light data */
            if (sensor_channel_get(sensor_node_dev, SENSOR_CHAN_LIGHT, &val) == 0) {
                float lux = sensor_value_to_double(&val);
                if (lux < light_low_threshold) {
                    printk("ALERT: Light level below threshold (%.2f lux)\n", lux);
                } else if (lux > light_high_threshold) {
                    printk("ALERT: Light level above threshold (%.2f lux)\n", lux);
                }
            }
            
            /* Get temperature data */
            if (sensor_channel_get(sensor_node_dev, SENSOR_CHAN_AMBIENT_TEMP, &val) == 0) {
                float temp = sensor_value_to_double(&val);
                if (temp < temp_low_threshold) {
                    printk("ALERT: Temperature below threshold (%.2f °C)\n", temp);
                } else if (temp > temp_high_threshold) {
                    printk("ALERT: Temperature above threshold (%.2f °C)\n", temp);
                }
            }
            
            /* Get soil moisture data */
            if (sensor_channel_get(sensor_node_dev, SENSOR_CHAN_HUMIDITY, &val) == 0) {
                if (val.val1 < soil_dry_threshold) {
                    printk("ALERT: Soil too dry (%d)\n", val.val1);
                } else if (val.val1 > soil_wet_threshold) {
                    printk("ALERT: Soil too wet (%d)\n", val.val1);
                }
            }
        } else {
            printk("Failed to fetch sensor data: %d\n", ret);
        }
        
        k_sleep(K_SECONDS(5));
    }
    
    printk("Sensor reading thread stopped\n");
}

/* External functions from sensor driver */
extern int sensor_node_ping(const struct device *dev);
extern int sensor_node_set_threshold(const struct device *dev, uint8_t sensor_type, 
                                     uint8_t alert_type, float threshold);

void main(void)
{
    printk("Smart Gardening System - Base Node\n");
    printk("--------------------------------\n");
    
    /* Get sensor node device */
    sensor_node_dev = DEVICE_DT_GET(DT_NODELABEL(sensor_node_0));
    if (!device_is_ready(sensor_node_dev)) {
        printk("Sensor node device not ready\n");
        return;
    }
    
    printk("Sensor node device ready\n");
    
    /* Start sensor reading thread */
    sensor_thread_running = true;
    sensor_thread_id = k_thread_create(&sensor_thread_data, sensor_thread_stack,
                                      K_THREAD_STACK_SIZEOF(sensor_thread_stack),
                                      sensor_thread, NULL, NULL, NULL,
                                      SENSOR_THREAD_PRIORITY, 0, K_NO_WAIT);
    
    if (sensor_thread_id == NULL) {
        printk("Failed to create sensor thread\n");
        return;
    }
    
    k_thread_name_set(sensor_thread_id, "sensor_thread");
    
    printk("System initialized and running\n");
    printk("Type 'help' for available commands\n");
}