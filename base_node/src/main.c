/**
 * @file main.c
 * @brief Main application for smart garden base node
 */
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/logging/log.h>
#include <zephyr/usb/usb_device.h>

LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);

/* Custom sensor channels */
enum {
    SENSOR_NODE_CHAN_SOIL_MOISTURE = SENSOR_CHAN_PRIV_START,
    SENSOR_NODE_CHAN_GAS_RESISTANCE
};

/* Thread stack size */
#define STACK_SIZE 1024

/* Thread priority */
#define PRIORITY 7

/* Stack area for the sensor thread */
K_THREAD_STACK_DEFINE(sensor_thread_stack, STACK_SIZE);
static struct k_thread sensor_thread_data;

/* Helper function to display sensor value */
static void print_sensor_value(const struct device *dev, enum sensor_channel chan, 
                              const char *name, const char *unit)
{
    struct sensor_value val;
    int ret = sensor_channel_get(dev, chan, &val);
    
    if (ret == 0) {
        LOG_INF("%s: %.2f %s", name, sensor_value_to_double(&val), unit);
    } else if (ret == -ENOTSUP) {
        LOG_INF("%s: Channel not supported", name);
    } else {
        LOG_ERR("%s: Error getting value: %d", name, ret);
    }
}

/* Sensor reading thread */
static void sensor_thread(void *p1, void *p2, void *p3)
{
    const struct device *sensor = p1;
    ARG_UNUSED(p2);
    ARG_UNUSED(p3);
    
    while (1) {
        /* Fetch all sensor values */
        int ret = sensor_sample_fetch(sensor);
        if (ret == 0) {
            /* Display sensor values */
            print_sensor_value(sensor, SENSOR_CHAN_AMBIENT_TEMP, "Temperature", "°C");
            print_sensor_value(sensor, SENSOR_CHAN_HUMIDITY, "Humidity", "%");
            print_sensor_value(sensor, SENSOR_CHAN_PRESS, "Pressure", "hPa");
            print_sensor_value(sensor, SENSOR_CHAN_LIGHT, "Light", "lux");
            print_sensor_value(sensor, SENSOR_NODE_CHAN_SOIL_MOISTURE, "Soil Moisture", "%");
            print_sensor_value(sensor, SENSOR_NODE_CHAN_GAS_RESISTANCE, "Gas Resistance", "Ohm");
            
            LOG_INF("----------------------------");
        } else {
            LOG_ERR("Error fetching sensor data: %d", ret);
        }
        
        /* Wait before next reading */
        k_sleep(K_SECONDS(5));
    }
}

int main(void)
{
    int ret;
    
    /* Initialize USB */
    ret = usb_enable(NULL);
    if (ret != 0) {
        LOG_ERR("Failed to enable USB: %d", ret);
        return 0;
    }
    
    /* Wait for USB to initialize */
    k_sleep(K_SECONDS(1));
    
    LOG_INF("=== Smart Garden Base Node ===");
    LOG_INF("USB Console enabled");
    
    /* Get sensor device */
    const struct device *sensor = DEVICE_DT_GET_ANY(sensor_node);
    if (!device_is_ready(sensor)) {
        LOG_ERR("Sensor device not ready");
        return 0;
    }
    
    LOG_INF("Sensor device ready - starting monitoring");
    
    /* Create sensor reading thread */
    k_thread_create(&sensor_thread_data, sensor_thread_stack,
                   K_THREAD_STACK_SIZEOF(sensor_thread_stack),
                   sensor_thread, (void *)sensor, NULL, NULL,
                   PRIORITY, 0, K_NO_WAIT);
    k_thread_name_set(&sensor_thread_data, "sensor_thread");
    
    return 0;
}