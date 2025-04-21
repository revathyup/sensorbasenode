/**
 * @file sensor_node_driver.c
 * @brief Zephyr sensor driver for the sensor node
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>
#include <string.h>

#include "protocol.h"
#include "sensor_node_driver.h"

LOG_MODULE_REGISTER(sensor_node, CONFIG_SENSOR_LOG_LEVEL);

// Size of the receive buffer for UART
#define UART_RX_BUF_SIZE 64

// Define the channel names for the sensor driver
enum sensor_channel_mapping {
    CHANNEL_LIGHT = 0,
    CHANNEL_TEMPERATURE = 1,
    CHANNEL_SOIL_MOISTURE = 2,
    CHANNEL_COUNT
};

// Driver data structure
struct sensor_node_data {
    // UART device
    const struct device *uart_dev;
    
    // Receive buffer
    uint8_t rx_buf[UART_RX_BUF_SIZE];
    uint8_t rx_pos;
    
    // Sensor values (scaled by 100)
    int32_t sensor_values[CHANNEL_COUNT];
    bool sensor_valid[CHANNEL_COUNT];
    
    // Last received message timestamp
    uint32_t last_rx_time;
    
    // UART mutex and semaphore
    struct k_mutex uart_mutex;
    struct k_sem rx_sem;
    
    // Sequence number for outgoing messages
    uint8_t sequence_number;
    
    // Interrupt status
    bool interrupt_high_temp;
    bool interrupt_low_soil;
    bool interrupt_low_light;
    bool interrupt_high_light;
    
    // Last error code
    uint8_t last_error_code;
    
    // Pending config request
    bool config_pending;
    enum config_param pending_param;
    int32_t pending_value;
    
    // Completed message
    message_t message;
    bool message_complete;
};

// Driver configuration structure
struct sensor_node_config {
    // UART device name
    const char *uart_name;
};

// Forward declarations for driver API
static int sensor_node_sample_fetch(const struct device *dev, enum sensor_channel chan);
static int sensor_node_channel_get(const struct device *dev, enum sensor_channel chan, struct sensor_value *val);
static int sensor_node_attr_set(const struct device *dev, enum sensor_channel chan, enum sensor_attribute attr, const struct sensor_value *val);
static int sensor_node_attr_get(const struct device *dev, enum sensor_channel chan, enum sensor_attribute attr, struct sensor_value *val);
static int sensor_node_trigger_set(const struct device *dev, const struct sensor_trigger *trig, sensor_trigger_handler_t handler);

// UART callback declaration
static void uart_callback(const struct device *dev, void *user_data);

// Protocol helper functions
static void process_message(const struct device *dev, const message_t *msg);
static bool send_message(const struct device *dev, message_t *msg);
static uint8_t calculate_checksum(const message_t *msg);
static bool validate_message(const message_t *msg);
static bool send_config_request(const struct device *dev, enum config_param param, int32_t value);

// Sensor driver API structure
static const struct sensor_driver_api sensor_node_driver_api = {
    .sample_fetch = sensor_node_sample_fetch,
    .channel_get = sensor_node_channel_get,
    .attr_set = sensor_node_attr_set,
    .attr_get = sensor_node_attr_get,
    .trigger_set = sensor_node_trigger_set,
};

// Initialize the driver
static int sensor_node_init(const struct device *dev)
{
    struct sensor_node_data *data = dev->data;
    const struct sensor_node_config *config = dev->config;
    
    // Get UART device
    data->uart_dev = device_get_binding(config->uart_name);
    if (data->uart_dev == NULL) {
        LOG_ERR("Failed to get UART device %s", config->uart_name);
        return -ENODEV;
    }
    
    // Initialize mutex and semaphore
    k_mutex_init(&data->uart_mutex);
    k_sem_init(&data->rx_sem, 0, 1);
    
    // Initialize driver data
    data->rx_pos = 0;
    data->sequence_number = 0;
    data->last_rx_time = 0;
    data->message_complete = false;
    data->config_pending = false;
    
    for (int i = 0; i < CHANNEL_COUNT; i++) {
        data->sensor_values[i] = 0;
        data->sensor_valid[i] = false;
    }
    
    data->interrupt_high_temp = false;
    data->interrupt_low_soil = false;
    data->interrupt_low_light = false;
    data->interrupt_high_light = false;
    data->last_error_code = 0;
    
    // Set up UART callback
    uart_callback_set(data->uart_dev, uart_callback, (void *)dev);
    
    LOG_INF("Sensor node driver initialized");
    return 0;
}

/**
 * @brief UART interrupt callback
 */
static void uart_callback(const struct device *uart_dev, void *user_data)
{
    const struct device *dev = (const struct device *)user_data;
    struct sensor_node_data *data = dev->data;
    uint8_t byte;
    
    // Handle received data
    while (uart_irq_update(uart_dev) && uart_irq_is_pending(uart_dev)) {
        if (uart_irq_rx_ready(uart_dev)) {
            // Read available data
            while (uart_fifo_read(uart_dev, &byte, 1) == 1) {
                // Check for start of new message
                if (data->rx_pos == 0 && byte == PROTOCOL_START_BYTE) {
                    // Start of a new message
                    data->rx_buf[data->rx_pos++] = byte;
                }
                // Continue filling buffer if we've already started a message
                else if (data->rx_pos > 0) {
                    // Store the byte
                    data->rx_buf[data->rx_pos++] = byte;
                    
                    // Check if we've received the message header
                    if (data->rx_pos == sizeof(message_header_t)) {
                        // Cast the buffer to a message structure to access header fields
                        message_header_t *header = (message_header_t *)data->rx_buf;
                        
                        // Check if we have a complete message (header + payload + checksum)
                        size_t expected_length = sizeof(message_header_t) + header->length + 1;
                        
                        if (data->rx_pos >= expected_length) {
                            // We have a complete message
                            memcpy(&data->message, data->rx_buf, expected_length);
                            data->message_complete = true;
                            data->rx_pos = 0; // Reset for next message
                            
                            // Validate and process the message
                            if (validate_message(&data->message)) {
                                process_message(dev, &data->message);
                                data->last_rx_time = k_uptime_get_32();
                            }
                        }
                    }
                    
                    // Prevent buffer overflow
                    if (data->rx_pos >= UART_RX_BUF_SIZE) {
                        // Reset if buffer gets full without a valid message
                        data->rx_pos = 0;
                    }
                }
            }
        }
        
        // Clear other interrupts
        if (uart_irq_tx_ready(uart_dev)) {
            uart_irq_tx_disable(uart_dev);
        }
    }
}

/**
 * @brief Process a received message
 */
static void process_message(const struct device *dev, const message_t *msg)
{
    if (!dev || !msg) return;
    
    struct sensor_node_data *data = dev->data;
    
    // Process based on message type
    switch (msg->header.message_type) {
        case MSG_SENSOR_DATA: {
            // Extract sensor data
            const sensor_data_t *sensor_data = &msg->payload.sensor;
            
            // Map sensor type to channel
            int channel = -1;
            switch (sensor_data->sensor_type) {
                case SENSOR_LIGHT:
                    channel = CHANNEL_LIGHT;
                    break;
                case SENSOR_TEMPERATURE:
                    channel = CHANNEL_TEMPERATURE;
                    break;
                case SENSOR_SOIL_MOISTURE:
                    channel = CHANNEL_SOIL_MOISTURE;
                    break;
                default:
                    LOG_WRN("Unknown sensor type: %d", sensor_data->sensor_type);
                    return;
            }
            
            // Store sensor value
            if (channel >= 0 && channel < CHANNEL_COUNT) {
                data->sensor_values[channel] = sensor_data->value;
                data->sensor_valid[channel] = true;
                
                // Log the update
                LOG_DBG("Updated sensor %d: %d", channel, sensor_data->value);
            }
            break;
        }
        
        case MSG_INTERRUPT: {
            const interrupt_data_t *int_data = &msg->payload.interrupt;
            
            // Set appropriate interrupt flag
            switch (int_data->interrupt_type) {
                case INT_HIGH_TEMPERATURE:
                    data->interrupt_high_temp = true;
                    LOG_INF("High temperature interrupt: %d (threshold: %d)",
                           int_data->value, int_data->threshold);
                    break;
                    
                case INT_LOW_SOIL_MOISTURE:
                    data->interrupt_low_soil = true;
                    LOG_INF("Low soil moisture interrupt: %d (threshold: %d)",
                           int_data->value, int_data->threshold);
                    break;
                    
                case INT_LOW_LIGHT:
                    data->interrupt_low_light = true;
                    LOG_INF("Low light interrupt: %d (threshold: %d)",
                           int_data->value, int_data->threshold);
                    break;
                    
                case INT_HIGH_LIGHT:
                    data->interrupt_high_light = true;
                    LOG_INF("High light interrupt: %d (threshold: %d)",
                           int_data->value, int_data->threshold);
                    break;
                    
                default:
                    LOG_WRN("Unknown interrupt type: %d", int_data->interrupt_type);
                    break;
            }
            break;
        }
        
        case MSG_CONFIG_RESPONSE: {
            // Check if this is a response to our pending config request
            if (data->config_pending && 
                msg->payload.config.param == data->pending_param) {
                
                // Clear pending flag
                data->config_pending = false;
                LOG_INF("Config parameter %d set to %d", 
                       msg->payload.config.param, msg->payload.config.value);
                
                // Signal completion
                k_sem_give(&data->rx_sem);
            }
            break;
        }
        
        case MSG_ERROR: {
            // Store error information
            data->last_error_code = msg->payload.error.error_code;
            LOG_ERR("Sensor node error: code=%d, context=%d", 
                   msg->payload.error.error_code,
                   msg->payload.error.context);
            break;
        }
        
        case MSG_ACK: {
            // ACK received, if waiting for config response, we can continue
            if (data->config_pending) {
                LOG_DBG("ACK received for config request");
            }
            break;
        }
        
        default:
            LOG_WRN("Unknown message type: %d", msg->header.message_type);
            break;
    }
}

/**
 * @brief Fetch a sample from the sensor
 */
static int sensor_node_sample_fetch(const struct device *dev, enum sensor_channel chan)
{
    struct sensor_node_data *data = dev->data;
    
    // No active fetch needed - data is updated by UART interrupt
    if (chan != SENSOR_CHAN_ALL && 
        chan != SENSOR_CHAN_LIGHT &&
        chan != SENSOR_CHAN_AMBIENT_TEMP &&
        chan != SENSOR_CHAN_HUMIDITY) {
        return -ENOTSUP;
    }
    
    // Check if we've received data recently (within last 5 seconds)
    uint32_t now = k_uptime_get_32();
    if (now - data->last_rx_time > 5000) {
        LOG_WRN("No recent data from sensor node (last: %d ms ago)",
               now - data->last_rx_time);
        return -ETIMEDOUT;
    }
    
    return 0;
}

/**
 * @brief Get a channel reading from the sensor
 */
static int sensor_node_channel_get(const struct device *dev, enum sensor_channel chan, struct sensor_value *val)
{
    struct sensor_node_data *data = dev->data;
    
    if (!val) {
        return -EINVAL;
    }
    
    // Map Zephyr sensor channel to our internal channel
    int idx = -1;
    switch (chan) {
        case SENSOR_CHAN_LIGHT:
            idx = CHANNEL_LIGHT;
            break;
            
        case SENSOR_CHAN_AMBIENT_TEMP:
            idx = CHANNEL_TEMPERATURE;
            break;
            
        case SENSOR_CHAN_HUMIDITY:  // We use humidity channel for soil moisture
            idx = CHANNEL_SOIL_MOISTURE;
            break;
            
        default:
            return -ENOTSUP;
    }
    
    // Check if we have valid data for this channel
    if (idx < 0 || idx >= CHANNEL_COUNT || !data->sensor_valid[idx]) {
        return -EINVAL;
    }
    
    // Convert the scaled integer value to sensor_value format
    // Our values are scaled by 100, so divide by 100 to get actual value
    val->val1 = data->sensor_values[idx] / 100;
    val->val2 = (data->sensor_values[idx] % 100) * 10000;  // Convert to micro units
    
    return 0;
}

/**
 * @brief Set an attribute on the sensor
 */
static int sensor_node_attr_set(const struct device *dev, enum sensor_channel chan, enum sensor_attribute attr, const struct sensor_value *val)
{
    if (!dev || !val) {
        return -EINVAL;
    }
    
    // Handle attribute setting
    if (attr != SENSOR_ATTR_UPPER_THRESH && 
        attr != SENSOR_ATTR_LOWER_THRESH &&
        attr != SENSOR_ATTR_SAMPLING_FREQUENCY) {
        return -ENOTSUP;
    }
    
    // Convert to scaled int32 value (multiply by 100)
    int32_t value = val->val1 * 100 + val->val2 / 10000;
    
    // Map channel and attribute to config parameter
    enum config_param param;
    switch (chan) {
        case SENSOR_CHAN_AMBIENT_TEMP:
            if (attr == SENSOR_ATTR_UPPER_THRESH) {
                param = CONFIG_TEMP_HIGH_THRESHOLD;
            } else {
                return -ENOTSUP;
            }
            break;
            
        case SENSOR_CHAN_HUMIDITY:  // We use humidity for soil moisture
            if (attr == SENSOR_ATTR_LOWER_THRESH) {
                param = CONFIG_SOIL_LOW_THRESHOLD;
            } else {
                return -ENOTSUP;
            }
            break;
            
        case SENSOR_CHAN_LIGHT:
            if (attr == SENSOR_ATTR_LOWER_THRESH) {
                param = CONFIG_LIGHT_LOW_THRESHOLD;
            } else if (attr == SENSOR_ATTR_UPPER_THRESH) {
                param = CONFIG_LIGHT_HIGH_THRESHOLD;
            } else {
                return -ENOTSUP;
            }
            break;
            
        case SENSOR_CHAN_ALL:
            if (attr == SENSOR_ATTR_SAMPLING_FREQUENCY) {
                // Convert Hz to ms for sampling rate
                param = CONFIG_SAMPLING_RATE;
                value = (int32_t)(1000.0f / ((float)val->val1 + (float)val->val2 / 1000000.0f));
            } else {
                return -ENOTSUP;
            }
            break;
            
        default:
            return -ENOTSUP;
    }
    
    // Send config request to sensor node
    return sensor_node_set_config(dev, param, value);
}

/**
 * @brief Get an attribute from the sensor
 */
static int sensor_node_attr_get(const struct device *dev, enum sensor_channel chan, enum sensor_attribute attr, struct sensor_value *val)
{
    if (!dev || !val) {
        return -EINVAL;
    }
    
    struct sensor_node_data *data = dev->data;
    
    // Implement basic attribute retrieval support
    // Note: This is a local implementation only as we don't have a way
    // to query the actual sensor node for its current configuration
    
    // Return locally cached values since we don't have a protocol message
    // to query the actual values from the sensor node
    switch (chan) {
        case SENSOR_CHAN_AMBIENT_TEMP:
            if (attr == SENSOR_ATTR_UPPER_THRESH) {
                // Use the last value we sent
                if (data->config_pending && data->pending_param == CONFIG_TEMP_HIGH_THRESHOLD) {
                    val->val1 = data->pending_value / 100;
                    val->val2 = (data->pending_value % 100) * 10000;
                    return 0;
                }
                return -ENODATA;
            }
            break;
            
        case SENSOR_CHAN_HUMIDITY:  // We use humidity for soil moisture
            if (attr == SENSOR_ATTR_LOWER_THRESH) {
                if (data->config_pending && data->pending_param == CONFIG_SOIL_LOW_THRESHOLD) {
                    val->val1 = data->pending_value / 100;
                    val->val2 = (data->pending_value % 100) * 10000;
                    return 0;
                }
                return -ENODATA;
            }
            break;
            
        case SENSOR_CHAN_LIGHT:
            if (attr == SENSOR_ATTR_LOWER_THRESH) {
                if (data->config_pending && data->pending_param == CONFIG_LIGHT_LOW_THRESHOLD) {
                    val->val1 = data->pending_value / 100;
                    val->val2 = (data->pending_value % 100) * 10000;
                    return 0;
                }
                return -ENODATA;
            }
            else if (attr == SENSOR_ATTR_UPPER_THRESH) {
                if (data->config_pending && data->pending_param == CONFIG_LIGHT_HIGH_THRESHOLD) {
                    val->val1 = data->pending_value / 100;
                    val->val2 = (data->pending_value % 100) * 10000;
                    return 0;
                }
                return -ENODATA;
            }
            break;
            
        case SENSOR_CHAN_ALL:
            if (attr == SENSOR_ATTR_SAMPLING_FREQUENCY) {
                if (data->config_pending && data->pending_param == CONFIG_SAMPLING_RATE) {
                    // Convert from ms to Hz
                    float frequency = 1000.0f / (float)data->pending_value;
                    val->val1 = (int32_t)frequency;
                    val->val2 = (int32_t)((frequency - (float)val->val1) * 1000000.0f);
                    return 0;
                }
                return -ENODATA;
            }
            break;
            
        default:
            return -ENOTSUP;
    }
    
    return -ENOTSUP;
}

/**
 * @brief Set a trigger on the sensor
 */
static int sensor_node_trigger_set(const struct device *dev, const struct sensor_trigger *trig, sensor_trigger_handler_t handler)
{
    // We don't currently support triggers
    return -ENOTSUP;
}

/**
 * @brief Send a message to the sensor node
 */
static bool send_message(const struct device *dev, message_t *msg)
{
    struct sensor_node_data *data = dev->data;
    
    if (!dev || !msg) {
        return false;
    }
    
    // Calculate message size
    size_t msg_size = sizeof(message_header_t) + msg->header.length + 1; // +1 for checksum
    
    // Lock UART
    k_mutex_lock(&data->uart_mutex, K_FOREVER);
    
    // Send the message
    for (size_t i = 0; i < msg_size; i++) {
        uart_poll_out(data->uart_dev, ((uint8_t *)msg)[i]);
    }
    
    // Unlock UART
    k_mutex_unlock(&data->uart_mutex);
    
    return true;
}

/**
 * @brief Calculate checksum for a message
 */
static uint8_t calculate_checksum(const message_t *msg)
{
    if (!msg) return 0;
    
    uint8_t sum = 0;
    const uint8_t *data = (const uint8_t *)msg;
    
    // Sum header bytes
    for (size_t i = 0; i < sizeof(message_header_t); i++) {
        sum += data[i];
    }
    
    // Sum payload bytes
    for (size_t i = 0; i < msg->header.length; i++) {
        sum += data[sizeof(message_header_t) + i];
    }
    
    return sum;
}

/**
 * @brief Validate a received message
 */
static bool validate_message(const message_t *msg)
{
    if (!msg) return false;
    
    // Check start byte
    if (msg->header.start_byte != PROTOCOL_START_BYTE) {
        LOG_WRN("Invalid start byte: 0x%02x", msg->header.start_byte);
        return false;
    }
    
    // Check length is reasonable
    if (msg->header.length > 32) {
        LOG_WRN("Invalid length: %d", msg->header.length);
        return false;
    }
    
    // Calculate and verify checksum
    uint8_t calculated_checksum = calculate_checksum(msg);
    uint8_t received_checksum = *((uint8_t *)msg + sizeof(message_header_t) + msg->header.length);
    
    if (calculated_checksum != received_checksum) {
        LOG_WRN("Checksum mismatch: calculated 0x%02x, received 0x%02x",
               calculated_checksum, received_checksum);
        return false;
    }
    
    return true;
}

/**
 * @brief Send a configuration request to the sensor node
 */
static bool send_config_request(const struct device *dev, enum config_param param, int32_t value)
{
    struct sensor_node_data *data = dev->data;
    
    // Prepare message
    message_t msg;
    memset(&msg, 0, sizeof(msg));
    
    msg.header.start_byte = PROTOCOL_START_BYTE;
    msg.header.message_type = MSG_CONFIG_REQUEST;
    msg.header.length = sizeof(config_data_t);
    msg.header.sequence = data->sequence_number++;
    
    msg.payload.config.param = param;
    msg.payload.config.value = value;
    
    msg.checksum = calculate_checksum(&msg);
    
    // Store pending request info
    data->config_pending = true;
    data->pending_param = param;
    data->pending_value = value;
    
    // Reset semaphore
    k_sem_reset(&data->rx_sem);
    
    // Send the message
    if (!send_message(dev, &msg)) {
        LOG_ERR("Failed to send config request");
        data->config_pending = false;
        return false;
    }
    
    // Wait for response with timeout
    int ret = k_sem_take(&data->rx_sem, K_MSEC(2000));
    if (ret != 0) {
        LOG_ERR("Timeout waiting for config response");
        data->config_pending = false;
        return false;
    }
    
    return true;
}

/**
 * @brief Configure the sensor node
 */
int sensor_node_set_config(const struct device *dev, enum config_param param, int32_t value)
{
    if (!dev) {
        return -EINVAL;
    }
    
    // Send config request
    if (!send_config_request(dev, param, value)) {
        return -EIO;
    }
    
    return 0;
}

/**
 * @brief Check if an interrupt has been triggered
 */
bool sensor_node_interrupt_is_triggered(const struct device *dev, enum interrupt_type type)
{
    if (!dev) {
        return false;
    }
    
    struct sensor_node_data *data = dev->data;
    
    switch (type) {
        case INT_HIGH_TEMPERATURE:
            return data->interrupt_high_temp;
            
        case INT_LOW_SOIL_MOISTURE:
            return data->interrupt_low_soil;
            
        case INT_LOW_LIGHT:
            return data->interrupt_low_light;
            
        case INT_HIGH_LIGHT:
            return data->interrupt_high_light;
            
        default:
            return false;
    }
}

/**
 * @brief Clear an interrupt
 */
int sensor_node_clear_interrupt(const struct device *dev, enum interrupt_type type)
{
    if (!dev) {
        return -EINVAL;
    }
    
    struct sensor_node_data *data = dev->data;
    
    switch (type) {
        case INT_HIGH_TEMPERATURE:
            data->interrupt_high_temp = false;
            break;
            
        case INT_LOW_SOIL_MOISTURE:
            data->interrupt_low_soil = false;
            break;
            
        case INT_LOW_LIGHT:
            data->interrupt_low_light = false;
            break;
            
        case INT_HIGH_LIGHT:
            data->interrupt_high_light = false;
            break;
            
        default:
            return -EINVAL;
    }
    
    return 0;
}

/**
 * @brief Get the last error code from the sensor node
 */
uint8_t sensor_node_get_last_error(const struct device *dev)
{
    if (!dev) {
        return 0;
    }
    
    struct sensor_node_data *data = dev->data;
    return data->last_error_code;
}

/**
 * @brief Get the device instance for the sensor node driver
 */
const struct device *sensor_node_driver_get_device(void)
{
    return DEVICE_DT_GET(DT_NODELABEL(sensor_node));
}

// Define device data and config
#define SENSOR_NODE_INIT(inst) \
    static struct sensor_node_data sensor_node_data_##inst; \
    static const struct sensor_node_config sensor_node_config_##inst = { \
        .uart_name = DT_LABEL(DT_ALIAS(sensor_uart)), \
    }; \
    DEVICE_DT_DEFINE(DT_NODELABEL(sensor_node), \
                    sensor_node_init, \
                    NULL, \
                    &sensor_node_data_##inst, \
                    &sensor_node_config_##inst, \
                    POST_KERNEL, \
                    CONFIG_SENSOR_INIT_PRIORITY, \
                    &sensor_node_driver_api);

// Instantiate the device
SENSOR_NODE_INIT(0);
