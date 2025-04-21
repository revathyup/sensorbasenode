/**
 * @file sensor_driver.c
 * @brief Zephyr driver implementation for sensor node
 */
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/util.h>
#include <zephyr/logging/log.h>
#include <string.h>

#include "sensor_driver.h"
#include "protocol.h"

LOG_MODULE_REGISTER(sensor_node, CONFIG_SENSOR_LOG_LEVEL);

// Device data structure
struct sensor_node_data {
    const struct device *uart_dev;
    const struct device *gpio_dev;
    struct gpio_callback gpio_cb;
    uint16_t light_value;
    uint16_t temperature_value;  // Fixed-point (0.1°C resolution)
    uint16_t moisture_value;
    bool data_ready;
    struct k_mutex lock;
    
    // Trigger support
    struct sensor_trigger_handler {
        sensor_trigger_handler_t handler;
        struct sensor_trigger trigger;
    } handlers[3];  // One handler per sensor type
};

// Communication buffer
static uint8_t rx_buffer[sizeof(protocol_packet_t)];
static int rx_buffer_index = 0;
static bool packet_received = false;
static protocol_packet_t rx_packet;

// Forward declarations
static void uart_rx_handler(const struct device *dev, void *user_data);
static void gpio_callback_handler(const struct device *dev, struct gpio_callback *cb, uint32_t pins);
static bool parse_packet(void);
static void process_packet(const struct device *dev);

/**
 * @brief Initialize the sensor node driver
 * 
 * @param dev Device instance
 * @return int 0 if successful, negative errno code otherwise
 */
int sensor_node_init(const struct device *dev) {
    struct sensor_node_data *data = dev->data;
    
    // Initialize mutex
    k_mutex_init(&data->lock);
    
    // Get UART device
    data->uart_dev = DEVICE_DT_GET(DT_ALIAS(sensor_uart));
    if (!device_is_ready(data->uart_dev)) {
        LOG_ERR("UART device not ready");
        return -ENODEV;
    }
    
    // Configure UART
    struct uart_config uart_cfg;
    uart_cfg.baudrate = 115200;
    uart_cfg.parity = UART_CFG_PARITY_NONE;
    uart_cfg.stop_bits = UART_CFG_STOP_BITS_1;
    uart_cfg.data_bits = UART_CFG_DATA_BITS_8;
    uart_cfg.flow_ctrl = UART_CFG_FLOW_CTRL_NONE;
    
    int ret = uart_configure(data->uart_dev, &uart_cfg);
    if (ret < 0) {
        LOG_ERR("Failed to configure UART: %d", ret);
        return ret;
    }
    
    // Set up UART callback
    uart_rx_enable(data->uart_dev, rx_buffer, sizeof(rx_buffer), 100);
    uart_callback_set(data->uart_dev, uart_rx_handler, (void *)dev);
    
    // Initialize GPIO for interrupt pin
    data->gpio_dev = DEVICE_DT_GET(DT_NODELABEL(gpio0));
    if (!device_is_ready(data->gpio_dev)) {
        LOG_ERR("GPIO device not ready");
        return -ENODEV;
    }
    
    // Configure interrupt pin
    ret = gpio_pin_configure(data->gpio_dev, CONFIG_INTERRUPT_PIN, 
                            GPIO_INPUT | GPIO_PULL_DOWN);
    if (ret < 0) {
        LOG_ERR("Failed to configure GPIO pin: %d", ret);
        return ret;
    }
    
    // Set up GPIO callback
    gpio_init_callback(&data->gpio_cb, gpio_callback_handler, BIT(CONFIG_INTERRUPT_PIN));
    gpio_add_callback(data->gpio_dev, &data->gpio_cb);
    gpio_pin_interrupt_configure(data->gpio_dev, CONFIG_INTERRUPT_PIN, 
                                GPIO_INT_EDGE_RISING);
    
    // Initialize sensor values
    data->light_value = 0;
    data->temperature_value = 0;
    data->moisture_value = 0;
    data->data_ready = false;
    
    // Send initial ping to verify communication
    protocol_packet_t tx_packet;
    tx_packet.start = PROTOCOL_START_BYTE;
    tx_packet.command = CMD_PING;
    tx_packet.data_length = 0;
    tx_packet.checksum = PROTOCOL_START_BYTE ^ CMD_PING ^ 0;
    
    uart_tx(data->uart_dev, (uint8_t *)&tx_packet, 4, SYS_FOREVER_MS);  // Start + cmd + len + checksum
    
    LOG_INF("Sensor node driver initialized");
    return 0;
}

/**
 * @brief Send a command to the sensor node
 * 
 * @param dev Device instance
 * @param cmd Command byte
 * @param data Data buffer
 * @param data_len Length of data buffer
 * @return int 0 if successful, negative errno code otherwise
 */
int sensor_node_send_command(const struct device *dev, uint8_t cmd, const uint8_t *data, uint8_t data_len) {
    struct sensor_node_data *drv_data = dev->data;
    
    protocol_packet_t tx_packet;
    tx_packet.start = PROTOCOL_START_BYTE;
    tx_packet.command = cmd;
    tx_packet.data_length = data_len;
    
    if (data && data_len > 0) {
        memcpy(tx_packet.data, data, data_len);
    }
    
    // Calculate checksum
    uint8_t checksum = tx_packet.start ^ tx_packet.command ^ tx_packet.data_length;
    for (int i = 0; i < data_len; i++) {
        checksum ^= tx_packet.data[i];
    }
    
    tx_packet.checksum = checksum;
    
    // Send packet over UART
    return uart_tx(drv_data->uart_dev, (uint8_t *)&tx_packet, data_len + 4, SYS_FOREVER_MS);
}

/**
 * @brief Sample data from all sensors
 * 
 * @param dev Device instance
 * @param chan Sensor channel to sample (or SENSOR_CHAN_ALL for all channels)
 * @return int 0 if successful, negative errno code otherwise
 */
int sensor_node_sample_fetch(const struct device *dev, enum sensor_channel chan) {
    struct sensor_node_data *data = dev->data;
    int ret;
    
    // Only fetch data if the requested channel is supported
    if (chan != SENSOR_CHAN_ALL && 
        chan != SENSOR_NODE_CHAN_LIGHT && 
        chan != SENSOR_NODE_CHAN_TEMPERATURE && 
        chan != SENSOR_NODE_CHAN_SOIL_MOISTURE) {
        return -ENOTSUP;
    }
    
    // Acquire lock to prevent concurrent access
    k_mutex_lock(&data->lock, K_FOREVER);
    
    uint8_t sensor_type;
    switch (chan) {
        case SENSOR_CHAN_ALL:
            sensor_type = SENSOR_ALL;
            break;
        case SENSOR_NODE_CHAN_LIGHT:
            sensor_type = SENSOR_LIGHT;
            break;
        case SENSOR_NODE_CHAN_TEMPERATURE:
            sensor_type = SENSOR_TEMPERATURE;
            break;
        case SENSOR_NODE_CHAN_SOIL_MOISTURE:
            sensor_type = SENSOR_SOIL_MOISTURE;
            break;
        default:
            k_mutex_unlock(&data->lock);
            return -ENOTSUP;
    }
    
    // Send command to read sensor data
    ret = sensor_node_send_command(dev, CMD_READ_SENSOR, &sensor_type, 1);
    if (ret < 0) {
        LOG_ERR("Failed to send read sensor command: %d", ret);
        k_mutex_unlock(&data->lock);
        return ret;
    }
    
    // Wait for response (with timeout)
    uint32_t start_time = k_uptime_get_32();
    while (!data->data_ready && (k_uptime_get_32() - start_time < 1000)) {
        k_sleep(K_MSEC(10));
    }
    
    if (!data->data_ready) {
        LOG_ERR("Timeout waiting for sensor data");
        k_mutex_unlock(&data->lock);
        return -ETIMEDOUT;
    }
    
    // Data was received and processed by the UART handler
    data->data_ready = false;
    k_mutex_unlock(&data->lock);
    
    return 0;
}

/**
 * @brief Get a sensor channel value
 * 
 * @param dev Device instance
 * @param chan Sensor channel
 * @param val Pointer to store the sensor value
 * @return int 0 if successful, negative errno code otherwise
 */
int sensor_node_channel_get(const struct device *dev, enum sensor_channel chan, struct sensor_value *val) {
    struct sensor_node_data *data = dev->data;
    
    k_mutex_lock(&data->lock, K_FOREVER);
    
    switch (chan) {
        case SENSOR_NODE_CHAN_LIGHT:
            val->val1 = data->light_value;
            val->val2 = 0;
            break;
            
        case SENSOR_NODE_CHAN_TEMPERATURE:
            // Convert fixed-point to sensor_value format (0.1°C resolution)
            val->val1 = data->temperature_value / 10;
            val->val2 = (data->temperature_value % 10) * 100000;
            break;
            
        case SENSOR_NODE_CHAN_SOIL_MOISTURE:
            val->val1 = data->moisture_value;
            val->val2 = 0;
            break;
            
        default:
            k_mutex_unlock(&data->lock);
            return -ENOTSUP;
    }
    
    k_mutex_unlock(&data->lock);
    return 0;
}

/**
 * @brief Set a sensor attribute
 * 
 * @param dev Device instance
 * @param chan Sensor channel
 * @param attr Sensor attribute
 * @param val Pointer to the attribute value
 * @return int 0 if successful, negative errno code otherwise
 */
int sensor_node_attr_set(const struct device *dev, enum sensor_channel chan, enum sensor_attribute attr, const struct sensor_value *val) {
    // This implementation supports setting thresholds as attributes
    if (attr != SENSOR_ATTR_UPPER_THRESH && attr != SENSOR_ATTR_LOWER_THRESH) {
        return -ENOTSUP;
    }
    
    uint8_t threshold_type = (attr == SENSOR_ATTR_UPPER_THRESH) ? 
                            THRESHOLD_TYPE_MAX : THRESHOLD_TYPE_MIN;
    
    uint16_t threshold = val->val1;
    
    // For temperature, we need to convert to our fixed-point format
    if (chan == SENSOR_NODE_CHAN_TEMPERATURE) {
        threshold = val->val1 * 10 + val->val2 / 100000;
    }
    
    switch (chan) {
        case SENSOR_NODE_CHAN_LIGHT:
            return sensor_node_set_light_threshold(dev, threshold, threshold_type);
            
        case SENSOR_NODE_CHAN_TEMPERATURE:
            return sensor_node_set_temperature_threshold(dev, threshold, threshold_type);
            
        case SENSOR_NODE_CHAN_SOIL_MOISTURE:
            return sensor_node_set_moisture_threshold(dev, threshold, threshold_type);
            
        default:
            return -ENOTSUP;
    }
    
    return 0;
}

/**
 * @brief Set a sensor trigger
 * 
 * @param dev Device instance
 * @param trig Pointer to the trigger configuration
 * @param handler Trigger handler function
 * @return int 0 if successful, negative errno code otherwise
 */
int sensor_node_trigger_set(const struct device *dev, const struct sensor_trigger *trig, sensor_trigger_handler_t handler) {
    struct sensor_node_data *data = dev->data;
    
    if (trig->type != SENSOR_TRIG_THRESHOLD) {
        return -ENOTSUP;
    }
    
    k_mutex_lock(&data->lock, K_FOREVER);
    
    switch (trig->chan) {
        case SENSOR_NODE_CHAN_LIGHT:
            data->handlers[0].trigger = *trig;
            data->handlers[0].handler = handler;
            break;
            
        case SENSOR_NODE_CHAN_TEMPERATURE:
            data->handlers[1].trigger = *trig;
            data->handlers[1].handler = handler;
            break;
            
        case SENSOR_NODE_CHAN_SOIL_MOISTURE:
            data->handlers[2].trigger = *trig;
            data->handlers[2].handler = handler;
            break;
            
        default:
            k_mutex_unlock(&data->lock);
            return -ENOTSUP;
    }
    
    k_mutex_unlock(&data->lock);
    return 0;
}

/**
 * @brief Set light threshold
 * 
 * @param dev Device instance
 * @param threshold Threshold value
 * @param threshold_type Type of threshold (min/max)
 * @return int 0 if successful, negative errno code otherwise
 */
int sensor_node_set_light_threshold(const struct device *dev, uint16_t threshold, uint8_t threshold_type) {
    config_data_t config;
    config.sensor_type = SENSOR_LIGHT;
    config.threshold = threshold;
    config.threshold_type = threshold_type;
    
    return sensor_node_send_command(dev, CMD_CONFIG_THRESHOLD, (uint8_t *)&config, sizeof(config));
}

/**
 * @brief Set temperature threshold
 * 
 * @param dev Device instance
 * @param threshold Threshold value
 * @param threshold_type Type of threshold (min/max)
 * @return int 0 if successful, negative errno code otherwise
 */
int sensor_node_set_temperature_threshold(const struct device *dev, uint16_t threshold, uint8_t threshold_type) {
    config_data_t config;
    config.sensor_type = SENSOR_TEMPERATURE;
    config.threshold = threshold;
    config.threshold_type = threshold_type;
    
    return sensor_node_send_command(dev, CMD_CONFIG_THRESHOLD, (uint8_t *)&config, sizeof(config));
}

/**
 * @brief Set moisture threshold
 * 
 * @param dev Device instance
 * @param threshold Threshold value
 * @param threshold_type Type of threshold (min/max)
 * @return int 0 if successful, negative errno code otherwise
 */
int sensor_node_set_moisture_threshold(const struct device *dev, uint16_t threshold, uint8_t threshold_type) {
    config_data_t config;
    config.sensor_type = SENSOR_SOIL_MOISTURE;
    config.threshold = threshold;
    config.threshold_type = threshold_type;
    
    return sensor_node_send_command(dev, CMD_CONFIG_THRESHOLD, (uint8_t *)&config, sizeof(config));
}

/**
 * @brief UART receive callback
 * 
 * @param dev UART device
 * @param user_data User data (device instance)
 */
static void uart_rx_handler(const struct device *dev, void *user_data) {
    const struct device *sensor_dev = (const struct device *)user_data;
    uint8_t c;
    
    while (uart_irq_update(dev) && uart_irq_is_pending(dev)) {
        if (uart_irq_rx_ready(dev)) {
            // Get received data
            if (uart_fifo_read(dev, &c, 1) == 1) {
                // Start of packet detection
                if (rx_buffer_index == 0 && c != PROTOCOL_START_BYTE) {
                    continue;
                }
                
                rx_buffer[rx_buffer_index++] = c;
                
                // Check if we have received a complete header (start byte, command, length)
                if (rx_buffer_index == 3) {
                    uint8_t data_length = rx_buffer[2];
                    
                    // Validate data length to prevent buffer overflow
                    if (data_length > PROTOCOL_MAX_DATA_LENGTH) {
                        // Invalid length, reset buffer
                        rx_buffer_index = 0;
                        continue;
                    }
                }
                
                // Check if we have received a complete packet (header + data + checksum)
                if (rx_buffer_index > 3) {
                    uint8_t data_length = rx_buffer[2];
                    if (rx_buffer_index == data_length + 4) {  // Start + command + length + data + checksum
                        // Parse packet
                        if (parse_packet()) {
                            packet_received = true;
                            process_packet(sensor_dev);
                        }
                        
                        // Reset buffer index for next packet
                        rx_buffer_index = 0;
                    }
                }
                
                // Buffer overflow protection
                if (rx_buffer_index >= sizeof(rx_buffer)) {
                    rx_buffer_index = 0;
                }
            }
        }
    }
}

/**
 * @brief Parse received data into packet structure
 * 
 * @return true if packet is valid, false otherwise
 */
static bool parse_packet(void) {
    memcpy(&rx_packet, rx_buffer, sizeof(protocol_packet_t));
    
    // Verify checksum
    uint8_t calculated_checksum = 0;
    for (int i = 0; i < rx_packet.data_length + 3; i++) {
        calculated_checksum ^= rx_buffer[i];
    }
    
    if (calculated_checksum != rx_packet.checksum) {
        LOG_ERR("Checksum verification failed");
        return false;
    }
    
    return true;
}

/**
 * @brief Process a received packet
 * 
 * @param dev Device instance
 */
static void process_packet(const struct device *dev) {
    struct sensor_node_data *data = dev->data;
    
    switch (rx_packet.command) {
        case CMD_ACK:
            // Acknowledgment received, nothing to do
            LOG_DBG("ACK received");
            break;
            
        case CMD_SENSOR_DATA: {
            k_mutex_lock(&data->lock, K_FOREVER);
            
            // Process sensor data
            if (rx_packet.data_length >= sizeof(sensor_data_t)) {
                sensor_data_t *sensor_data = (sensor_data_t *)rx_packet.data;
                int num_sensors = rx_packet.data_length / sizeof(sensor_data_t);
                
                // Process each sensor data
                for (int i = 0; i < num_sensors; i++) {
                    switch (sensor_data[i].sensor_type) {
                        case SENSOR_LIGHT:
                            data->light_value = sensor_data[i].value;
                            LOG_DBG("Light value: %u", data->light_value);
                            break;
                            
                        case SENSOR_TEMPERATURE:
                            data->temperature_value = sensor_data[i].value;
                            LOG_DBG("Temperature value: %u (0.1°C)", data->temperature_value);
                            break;
                            
                        case SENSOR_SOIL_MOISTURE:
                            data->moisture_value = sensor_data[i].value;
                            LOG_DBG("Moisture value: %u", data->moisture_value);
                            break;
                            
                        default:
                            LOG_WRN("Unknown sensor type: %u", sensor_data[i].sensor_type);
                            break;
                    }
                }
                
                data->data_ready = true;
            }
            
            k_mutex_unlock(&data->lock);
            break;
        }
        
        case CMD_ERROR: {
            if (rx_packet.data_length > 0) {
                LOG_ERR("Error response received: %u", rx_packet.data[0]);
            } else {
                LOG_ERR("Unknown error response received");
            }
            break;
        }
        
        case CMD_INTERRUPT: {
            if (rx_packet.data_length >= sizeof(interrupt_data_t)) {
                interrupt_data_t *interrupt_data = (interrupt_data_t *)rx_packet.data;
                
                LOG_INF("Interrupt received: sensor=%u, value=%u, threshold=%u, type=%u",
                        interrupt_data->sensor_type, interrupt_data->value,
                        interrupt_data->threshold, interrupt_data->threshold_type);
                
                // Update sensor value
                k_mutex_lock(&data->lock, K_FOREVER);
                
                switch (interrupt_data->sensor_type) {
                    case SENSOR_LIGHT:
                        data->light_value = interrupt_data->value;
                        if (data->handlers[0].handler) {
                            data->handlers[0].handler(dev, &data->handlers[0].trigger);
                        }
                        break;
                        
                    case SENSOR_TEMPERATURE:
                        data->temperature_value = interrupt_data->value;
                        if (data->handlers[1].handler) {
                            data->handlers[1].handler(dev, &data->handlers[1].trigger);
                        }
                        break;
                        
                    case SENSOR_SOIL_MOISTURE:
                        data->moisture_value = interrupt_data->value;
                        if (data->handlers[2].handler) {
                            data->handlers[2].handler(dev, &data->handlers[2].trigger);
                        }
                        break;
                }
                
                k_mutex_unlock(&data->lock);
            }
            break;
        }
        
        default:
            LOG_WRN("Unknown command received: %u", rx_packet.command);
            break;
    }
}

/**
 * @brief GPIO interrupt callback
 * 
 * @param dev GPIO device
 * @param cb GPIO callback structure
 * @param pins GPIO pins that triggered the interrupt
 */
static void gpio_callback_handler(const struct device *dev, struct gpio_callback *cb, uint32_t pins) {
    struct sensor_node_data *data = CONTAINER_OF(cb, struct sensor_node_data, gpio_cb);
    
    // GPIO interrupt received, fetch updated sensor values
    LOG_INF("Interrupt signal received from sensor node");
    
    // The actual handling will be done when the CMD_INTERRUPT packet is received
}

// Define the driver API structure
static const struct sensor_driver_api sensor_node_api = {
    .sample_fetch = sensor_node_sample_fetch,
    .channel_get = sensor_node_channel_get,
    .attr_set = sensor_node_attr_set,
    .trigger_set = sensor_node_trigger_set
};

// Define device data
static struct sensor_node_data sensor_node_data;

// Define device initialization
DEVICE_DEFINE(sensor_node, "sensor_node", sensor_node_init, NULL,
             &sensor_node_data, NULL, POST_KERNEL,
             CONFIG_SENSOR_INIT_PRIORITY, &sensor_node_api);
