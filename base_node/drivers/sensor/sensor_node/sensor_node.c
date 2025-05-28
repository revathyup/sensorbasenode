/**
 * @file sensor_node.c
 * @brief Sensor driver for communicating with sensor node via UART
 */
#include <math.h>  // Add for isfinite()

#define DT_DRV_COMPAT zephyr_sensor_node

// Add initialization priority
#if !defined(CONFIG_SENSOR_INIT_PRIORITY)
#define CONFIG_SENSOR_INIT_PRIORITY 70
#endif

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/ring_buffer.h>
#include <string.h>

LOG_MODULE_REGISTER(sensor_node, LOG_LEVEL_INF);

/* Protocol definitions */
#define PROTOCOL_START_BYTE     0xAA  // Change this to match sensor node
#define PROTOCOL_MAX_DATA_LEN   64

/* Command types */
#define CMD_SENSOR_DATA     0x01
#define CMD_ALERT          0x02
#define CMD_PING           0x03
#define CMD_ACK            0x04

/* Sensor types - update these */
#define SENSOR_LIGHT        0x01  // TSL2591
#define SENSOR_TEMP         0x02  // MCP9700
#define SENSOR_BME680       0x03  // BME680

/* Buffer sizes */
#define RX_BUFFER_SIZE 128
#define TX_BUFFER_SIZE 64

/* Custom channels - update enum */
enum sensor_node_channels {
    SENSOR_NODE_CHAN_GAS_RESISTANCE = SENSOR_CHAN_PRIV_START
};

/* Packet structure */
typedef struct {
    uint8_t start;
    uint8_t command;
    uint8_t length;
    uint8_t data[PROTOCOL_MAX_DATA_LEN];
    uint8_t checksum;
} packet_t;

/* Driver data structure */
struct sensor_node_data {
    /* Cached sensor values */
    struct sensor_value temperature;
    struct sensor_value humidity;
    struct sensor_value pressure;
    struct sensor_value light;
    struct sensor_value gas_resistance;
    bool data_ready;
    
    /* UART communication */
    const struct device *uart_dev;
    struct ring_buf rx_rb;
    struct ring_buf tx_rb;
    uint8_t rx_buffer[RX_BUFFER_SIZE];
    uint8_t tx_buffer[TX_BUFFER_SIZE];

    /* Mutex to protect data access */
    struct k_mutex mutex;
};

/* Calculate checksum for packet */
static uint8_t calculate_checksum(const packet_t *packet)
{
    uint8_t checksum = 0;
    checksum ^= packet->start;
    checksum ^= packet->command;
    checksum ^= packet->length;
    for (int i = 0; i < packet->length; i++) {
        checksum ^= packet->data[i];
    }
    return checksum;
}

/* UART callback for interrupt-driven mode */
static void uart_callback(const struct device *dev, void *user_data)
{
    struct sensor_node_data *data = (struct sensor_node_data *)user_data;

    if (!uart_irq_update(dev)) {
        return;
    }

    /* Handle RX */
    if (uart_irq_rx_ready(dev)) {
        uint8_t c;
        while (uart_fifo_read(dev, &c, 1) == 1) {
            ring_buf_put(&data->rx_rb, &c, 1);
        }
    }

    /* Handle TX */
    if (uart_irq_tx_ready(dev)) {
        uint8_t c;
        int bytes_read = ring_buf_get(&data->tx_rb, &c, 1);
        if (bytes_read == 0) {
            /* No more data to send, disable TX interrupt */
            uart_irq_tx_disable(dev);
        } else {
            uart_fifo_fill(dev, &c, 1);
        }
    }
}

/* Write a byte to the UART */
static void uart_write_byte(struct sensor_node_data *data, uint8_t byte)
{
    ring_buf_put(&data->tx_rb, &byte, 1);
    uart_irq_tx_enable(data->uart_dev);
}

/* Send a packet over UART */
static void send_packet(struct sensor_node_data *data, const packet_t *packet)
{
    uart_write_byte(data, packet->start);
    uart_write_byte(data, packet->command);
    uart_write_byte(data, packet->length);
    
    for (int i = 0; i < packet->length; i++) {
        uart_write_byte(data, packet->data[i]);
    }
    
    uart_write_byte(data, packet->checksum);
}

/* Try to read a packet from the ring buffer */
static bool read_packet(struct sensor_node_data *data, packet_t *packet)
{
    /* Try to find the start byte */
    uint8_t byte;
    while (ring_buf_size_get(&data->rx_rb) > 0) {
        ring_buf_get(&data->rx_rb, &byte, 1);
        if (byte == PROTOCOL_START_BYTE) {
            packet->start = byte;
            break;
        }
    }
    
    /* Check if we have enough data for a complete packet */
    if (ring_buf_size_get(&data->rx_rb) < 2) {
        return false;  /* Need at least command and length */
    }
    
    /* Read command and length */
    ring_buf_get(&data->rx_rb, &packet->command, 1);
    ring_buf_get(&data->rx_rb, &packet->length, 1);
    
    /* Validate packet length */
    if (packet->length > PROTOCOL_MAX_DATA_LEN) {
        LOG_ERR("Invalid packet length: %d", packet->length);
        return false;
    }
    
    /* Check if we have enough data for the rest of the packet */
    if (ring_buf_size_get(&data->rx_rb) < packet->length + 1) {
        return false;  /* Not enough data for payload + checksum */
    }
    
    /* Read data and checksum */
    ring_buf_get(&data->rx_rb, packet->data, packet->length);
    ring_buf_get(&data->rx_rb, &packet->checksum, 1);
    
    /* Validate checksum */
    uint8_t calculated_checksum = calculate_checksum(packet);
    if (calculated_checksum != packet->checksum) {
        LOG_ERR("Checksum mismatch: expected 0x%02X, got 0x%02X",
               calculated_checksum, packet->checksum);
        return false;
    }
    
    return true;
}

/* Process a sensor data packet - Update BME680 handling */
static void process_sensor_data(struct sensor_node_data *data, const packet_t *packet)
{
    if (packet->length == 0) {
        return;
    }

    k_mutex_lock(&data->mutex, K_FOREVER);

    switch (packet->data[0]) {
    case SENSOR_LIGHT:  // TSL2591
        if (packet->length >= 11) {
            float lux;
            memcpy(&lux, &packet->data[7], sizeof(float));
            sensor_value_from_double(&data->light, lux);
            LOG_DBG("Light: %.2f lux", (double)lux);
        }
        break;
        
    case SENSOR_TEMP:  // MCP9700
        if (packet->length >= sizeof(float) + 1) {
            float temp;
            memcpy(&temp, &packet->data[1], sizeof(float));
            sensor_value_from_double(&data->temperature, temp);
            LOG_DBG("MCP9700 Temp: %.1f C", (double)temp);
        }
        break;

    case SENSOR_BME680:
        if (packet->length >= sizeof(float) * 4 + 1) {
            float temp, humidity, pressure, gas;
            
            // Print the entire packet for debugging
            LOG_HEXDUMP_DBG(packet->data, packet->length, "BME680 Raw Packet:");
            LOG_DBG("Packet length: %d (expected %d)", 
                    packet->length, sizeof(float) * 4 + 1);
            
            // Extract and verify each value
            memcpy(&temp, &packet->data[1], sizeof(float));
            memcpy(&humidity, &packet->data[5], sizeof(float));
            memcpy(&pressure, &packet->data[9], sizeof(float));
            memcpy(&gas, &packet->data[13], sizeof(float));
            
            LOG_DBG("BME680 Raw Values:");
            LOG_DBG("  Temperature: %f", (double)temp);
            LOG_DBG("  Humidity: %f", (double)humidity);
            LOG_DBG("  Pressure: %f", (double)pressure);
            LOG_DBG("  Gas: %f", (double)gas);
            
            // Store valid values with range checks
            if (isfinite(temp) && temp > -40.0f && temp < 85.0f) {
                sensor_value_from_double(&data->temperature, temp);
                LOG_INF("Temperature: %.2f C", (double)temp);
            } else {
                LOG_WRN("Invalid temperature: %f", (double)temp);
            }
            
            if (isfinite(humidity) && humidity >= 0.0f && humidity <= 100.0f) {
                sensor_value_from_double(&data->humidity, humidity);
                LOG_INF("Humidity: %.2f %%", (double)humidity);
            } else {
                LOG_WRN("Invalid humidity: %f", (double)humidity);
            }
            
            if (isfinite(pressure) && pressure >= 300.0f && pressure <= 1100.0f) {
                sensor_value_from_double(&data->pressure, pressure);
                LOG_INF("Pressure: %.2f hPa", (double)pressure);
            } else {
                LOG_WRN("Invalid pressure: %f", (double)pressure);
            }
            
            if (isfinite(gas) && gas > 0.0f) {
                sensor_value_from_double(&data->gas_resistance, gas);
                LOG_INF("Gas resistance: %.2f Ohm", (double)gas);
            } else {
                LOG_WRN("Invalid gas resistance: %f", (double)gas);
            }
        } else {
            LOG_WRN("BME680 data packet too short: %d", packet->length);
        }
        break;
        
    default:
        break;
    }

    data->data_ready = true;
    k_mutex_unlock(&data->mutex);
}

/* Implement the sensor_sample_fetch function */
static int sensor_node_sample_fetch(const struct device *dev, enum sensor_channel chan)
{
    struct sensor_node_data *data = dev->data;
    k_timeout_t timeout_duration = K_MSEC(500);  // Define timeout as k_timeout_t
    
    LOG_DBG("Fetching sensor data");
    
    k_mutex_lock(&data->mutex, K_FOREVER);

    // Clear old data
    ring_buf_reset(&data->rx_rb);
    data->data_ready = false;
    
    // Try up to 3 times
    for (int retry = 0; retry < 3; retry++) {
        // Send ping packet
        packet_t ping = {
            .start = PROTOCOL_START_BYTE,
            .command = CMD_PING,
            .length = 0,
            .checksum = 0
        };
        ping.checksum = calculate_checksum(&ping);
        
        send_packet(data, &ping);
        LOG_DBG("Sent ping packet, attempt %d", retry + 1);

        // Wait for response with timeout
        int64_t end_time = k_uptime_get() + k_ticks_to_ms_floor64(timeout_duration.ticks);
        bool got_data = false;

        while (k_uptime_get() < end_time) {
            packet_t rx_packet;
            if (read_packet(data, &rx_packet)) {
                if (rx_packet.command == CMD_SENSOR_DATA) {
                    process_sensor_data(data, &rx_packet);
                    got_data = true;
                    break;
                } else if (rx_packet.command == CMD_ACK) {
                    LOG_DBG("Got ACK, waiting for data");
                }
            }
            k_sleep(K_MSEC(10));
        }

        if (got_data) {
            k_mutex_unlock(&data->mutex);
            return 0;
        }

        LOG_WRN("Retry %d failed", retry + 1);
        k_sleep(K_MSEC(100));  // Wait before retry
    }

    LOG_ERR("No sensor data received after 3 attempts");
    k_mutex_unlock(&data->mutex);
    return -ETIMEDOUT;
}

/* Implement the sensor_channel_get function for the Sensors API */
static int sensor_node_channel_get(const struct device *dev, enum sensor_channel chan,
                                 struct sensor_value *val)
{
    struct sensor_node_data *data = dev->data;
    int ret = 0;
    
    if (!data->data_ready) {
        return -ENODATA;
    }
    
    k_mutex_lock(&data->mutex, K_FOREVER);
    
    switch (chan) {
    case SENSOR_CHAN_AMBIENT_TEMP:
        *val = data->temperature;
        break;
        
    case SENSOR_CHAN_HUMIDITY:
        *val = data->humidity;
        break;
        
    case SENSOR_CHAN_PRESS:
        *val = data->pressure;
        break;
        
    case SENSOR_CHAN_LIGHT:
        *val = data->light;
        break;
        
    case SENSOR_NODE_CHAN_GAS_RESISTANCE:
        *val = data->gas_resistance;
        break;
        
    default:
        ret = -ENOTSUP;
        break;
    }
    
    k_mutex_unlock(&data->mutex);
    return ret;
}

/* Define the Sensors API function table */
static const struct sensor_driver_api sensor_node_api = {
    .sample_fetch = sensor_node_sample_fetch,
    .channel_get = sensor_node_channel_get,
};

/* Driver initialization function */
static int sensor_node_init(const struct device *dev)
{
    struct sensor_node_data *data = dev->data;
    
    /* Initialize mutex */
    k_mutex_init(&data->mutex);
    
    /* Initialize ring buffers */
    ring_buf_init(&data->rx_rb, sizeof(data->rx_buffer), data->rx_buffer);
    ring_buf_init(&data->tx_rb, sizeof(data->tx_buffer), data->tx_buffer);
    
    /* Get UART device from devicetree */
    data->uart_dev = DEVICE_DT_GET(DT_NODELABEL(uart0));
    if (!device_is_ready(data->uart_dev)) {
        LOG_ERR("UART device not ready");
        return -ENODEV;
    }
    
    /* Set up UART callback */
    uart_irq_callback_user_data_set(data->uart_dev, uart_callback, data);
    uart_irq_rx_enable(data->uart_dev);
    
    /* Initialize sensor values */
    data->data_ready = false;
    
    LOG_INF("Sensor node driver initialized");
    return 0;
}

/* Define driver data */
static struct sensor_node_data sensor_node_data_0;

/* Register the driver */
DEVICE_DT_INST_DEFINE(0,
                      sensor_node_init,
                      NULL,
                      &sensor_node_data_0,
                      NULL,
                      POST_KERNEL,
                      CONFIG_SENSOR_INIT_PRIORITY,
                      &sensor_node_api);
