/**
 * @file sensor_node_driver.c
 * @brief Zephyr driver implementation for interfacing with the RP2040 sensor node
 */

#include <zephyr/kernel.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/logging/log.h>
#include <string.h>

#include "sensor_node_driver.h"
#include "../../protocol.h"

LOG_MODULE_REGISTER(sensor_node, CONFIG_SENSOR_LOG_LEVEL);

/* UART callback function */
static void uart_callback(const struct device *dev, void *user_data)
{
    struct sensor_node_data *data = (struct sensor_node_data *)user_data;
    uint8_t received_byte;
    
    /* Read the received byte */
    if (uart_fifo_read(dev, &received_byte, 1) == 1) {
        /* Process the received byte according to protocol */
        if (data->rx_index == 0 && received_byte != PROTOCOL_START_BYTE) {
            /* Ignore bytes until start byte is found */
            return;
        }
        
        /* Store byte in receive buffer */
        data->rx_buffer[data->rx_index++] = received_byte;
        
        /* Check if we have received a complete packet */
        if (data->rx_index >= 3) {  // We have start, command, and length
            uint8_t *buffer = data->rx_buffer;
            uint8_t length = buffer[2];
            
            if (data->rx_index >= 4 + length) {  // Start + cmd + length + data + checksum
                /* We have a complete packet */
                protocol_packet_t *packet = (protocol_packet_t *)buffer;
                
                /* Calculate and verify checksum */
                uint8_t checksum = 0;
                for (int i = 0; i < 3 + length; i++) {
                    checksum ^= buffer[i];
                }
                
                if (checksum == buffer[3 + length]) {
                    /* Valid packet, copy to rx_packet */
                    memcpy(&data->rx_packet, packet, sizeof(protocol_packet_t));
                    data->packet_ready = true;
                    
                    /* Process the packet based on command */
                    k_mutex_lock(&data->data_mutex, K_FOREVER);
                    
                    switch (packet->command) {
                    case CMD_SENSOR_DATA:
                        /* Extract sensor data based on type */
                        if (packet->data[0] == SENSOR_LIGHT) {
                            light_data_t *light_data = (light_data_t *)packet->data;
                            data->light_full = light_data->full;
                            data->light_ir = light_data->ir;
                            data->light_visible = light_data->visible;
                            data->light_lux = light_data->lux;
                            LOG_DBG("Light: %.2f lux", data->light_lux);
                        } else if (packet->data[0] == SENSOR_TEMPERATURE) {
                            data->temperature = *((float *)&packet->data[1]);
                            LOG_DBG("Temperature: %.2f °C", data->temperature);
                        } else if (packet->data[0] == SENSOR_SOIL_MOISTURE) {
                            data->soil_moisture = *((uint16_t *)&packet->data[1]);
                            data->soil_moisture_pct = packet->data[3];
                            data->soil_temperature = *((float *)&packet->data[4]);
                            LOG_DBG("Soil moisture: %u (%u%%)", data->soil_moisture, data->soil_moisture_pct);
                        }
                        break;
                        
                    case CMD_ALERT:
                        /* Process alerts */
                        LOG_WRN("Alert received: type=%u", packet->data[0]);
                        break;
                        
                    case CMD_ACK:
                        LOG_DBG("ACK received");
                        break;
                        
                    default:
                        LOG_WRN("Unknown command: %u", packet->command);
                        break;
                    }
                    
                    k_mutex_unlock(&data->data_mutex);
                }
                
                /* Reset receive index for next packet */
                data->rx_index = 0;
            }
        }
    }
}

/* Calculate checksum for a packet */
static uint8_t calculate_checksum(protocol_packet_t *packet)
{
    uint8_t checksum = 0;
    uint8_t *data = (uint8_t *)packet;
    
    for (int i = 0; i < 3 + packet->length; i++) {
        checksum ^= data[i];
    }
    
    return checksum;
}

/* Send a packet to the sensor node */
static int send_packet(const struct device *uart_dev, protocol_packet_t *packet)
{
    packet->start = PROTOCOL_START_BYTE;
    packet->checksum = calculate_checksum(packet);
    
    /* Send the packet over UART */
    for (int i = 0; i < 4 + packet->length; i++) {
        uart_poll_out(uart_dev, ((uint8_t *)packet)[i]);
    }
    
    return 0;
}

/* Sensor driver initialization */
static int sensor_node_init(const struct device *dev)
{
    struct sensor_node_data *data = dev->data;
    const struct sensor_node_config *config = dev->config;
    
    /* Initialize mutex */
    k_mutex_init(&data->data_mutex);
    
    /* Initialize UART */
    data->uart_dev = DEVICE_DT_GET(DT_NODELABEL(uart1));
    if (!device_is_ready(data->uart_dev)) {
        LOG_ERR("UART device not ready");
        return -ENODEV;
    }
    
    if (uart_configure(data->uart_dev, &config->uart_cfg) != 0) {
        LOG_ERR("Failed to configure UART");
        return -EIO;
    }
    
    /* Set up UART callback */
    uart_irq_callback_user_data_set(data->uart_dev, uart_callback, data);
    uart_irq_rx_enable(data->uart_dev);
    
    /* Initialize data fields */
    data->rx_index = 0;
    data->packet_ready = false;
    data->light_lux = 0.0f;
    data->temperature = 0.0f;
    data->soil_moisture = 0;
    
    /* Default thresholds */
    data->light_low_threshold = 50.0f;    /* 50 lux */
    data->light_high_threshold = 10000.0f; /* 10,000 lux */
    data->temp_low_threshold = 10.0f;     /* 10°C */
    data->temp_high_threshold = 30.0f;    /* 30°C */
    data->soil_dry_threshold = 300;       /* 30% */
    data->soil_wet_threshold = 700;       /* 70% */
    
    LOG_INF("Sensor node driver initialized");
    return 0;
}

/* Send ping to sensor node */
int sensor_node_ping(const struct device *dev)
{
    struct sensor_node_data *data = dev->data;
    protocol_packet_t packet = {0};
    
    packet.command = CMD_PING;
    packet.length = 0;
    
    LOG_DBG("Sending ping to sensor node");
    return send_packet(data->uart_dev, &packet);
}

/* Request sensor data from the sensor node */
int sensor_node_fetch_data(const struct device *dev)
{
    struct sensor_node_data *data = dev->data;
    protocol_packet_t packet = {0};
    
    packet.command = CMD_SENSOR_DATA;
    packet.length = 1;
    packet.data[0] = 0xFF;  /* Request all sensor data */
    
    LOG_DBG("Requesting sensor data");
    return send_packet(data->uart_dev, &packet);
}

/* Set threshold for sensor alerts */
int sensor_node_set_threshold(const struct device *dev, uint8_t sensor_type, 
                             uint8_t alert_type, float threshold)
{
    struct sensor_node_data *data = dev->data;
    protocol_packet_t packet = {0};
    
    packet.command = CMD_SENSOR_CONFIG;
    packet.length = 6;  /* sensor_type + alert_type + 4 bytes for float */
    packet.data[0] = sensor_type;
    packet.data[1] = alert_type;
    memcpy(&packet.data[2], &threshold, sizeof(float));
    
    /* Update local thresholds */
    k_mutex_lock(&data->data_mutex, K_FOREVER);
    
    if (sensor_type == SENSOR_LIGHT) {
        if (alert_type == ALERT_LIGHT_LOW) {
            data->light_low_threshold = threshold;
        } else if (alert_type == ALERT_LIGHT_HIGH) {
            data->light_high_threshold = threshold;
        }
    } else if (sensor_type == SENSOR_TEMPERATURE) {
        if (alert_type == ALERT_TEMP_LOW) {
            data->temp_low_threshold = threshold;
        } else if (alert_type == ALERT_TEMP_HIGH) {
            data->temp_high_threshold = threshold;
        }
    } else if (sensor_type == SENSOR_SOIL_MOISTURE) {
        if (alert_type == ALERT_SOIL_DRY) {
            data->soil_dry_threshold = (uint16_t)threshold;
        } else if (alert_type == ALERT_SOIL_WET) {
            data->soil_wet_threshold = (uint16_t)threshold;
        }
    }
    
    k_mutex_unlock(&data->data_mutex);
    
    LOG_DBG("Setting threshold: sensor=%u, alert=%u, value=%.2f", 
            sensor_type, alert_type, threshold);
    return send_packet(data->uart_dev, &packet);
}

/* SENSOR API IMPLEMENTATION */

static int sensor_node_sample_fetch(const struct device *dev, enum sensor_channel chan)
{
    /* Request updated sensor data */
    return sensor_node_fetch_data(dev);
}

static int sensor_node_channel_get(const struct device *dev,
                                  enum sensor_channel chan,
                                  struct sensor_value *val)
{
    struct sensor_node_data *data = dev->data;
    int ret = 0;
    
    k_mutex_lock(&data->data_mutex, K_FOREVER);
    
    switch (chan) {
    case SENSOR_NODE_CHANNEL_LIGHT_LUX:
        sensor_value_from_double(val, data->light_lux);
        break;
        
    case SENSOR_NODE_CHANNEL_LIGHT_FULL:
        val->val1 = data->light_full;
        val->val2 = 0;
        break;
        
    case SENSOR_NODE_CHANNEL_LIGHT_IR:
        val->val1 = data->light_ir;
        val->val2 = 0;
        break;
        
    case SENSOR_NODE_CHANNEL_LIGHT_VISIBLE:
        val->val1 = data->light_visible;
        val->val2 = 0;
        break;
        
    case SENSOR_NODE_CHANNEL_TEMPERATURE:
        sensor_value_from_double(val, data->temperature);
        break;
        
    case SENSOR_NODE_CHANNEL_SOIL_MOISTURE:
        val->val1 = data->soil_moisture;
        val->val2 = data->soil_moisture_pct * 10000; /* Convert to humidity format */
        break;
        
    case SENSOR_NODE_CHANNEL_SOIL_TEMPERATURE:
        sensor_value_from_double(val, data->soil_temperature);
        break;
        
    default:
        ret = -ENOTSUP;
        break;
    }
    
    k_mutex_unlock(&data->data_mutex);
    return ret;
}

/* Sensor driver API structure */
const struct sensor_driver_api sensor_node_driver_api = {
    .sample_fetch = sensor_node_sample_fetch,
    .channel_get = sensor_node_channel_get,
};

/* Driver instantiation */
#define SENSOR_NODE_INIT(inst)                                               \
    static struct sensor_node_data sensor_node_data_##inst;                  \
                                                                             \
    static const struct sensor_node_config sensor_node_config_##inst = {     \
        .uart_cfg = {                                                        \
            .baudrate = 115200,                                              \
            .parity = UART_CFG_PARITY_NONE,                                  \
            .stop_bits = UART_CFG_STOP_BITS_1,                               \
            .data_bits = UART_CFG_DATA_BITS_8,                               \
            .flow_ctrl = UART_CFG_FLOW_CTRL_NONE,                            \
        },                                                                   \
        .tx_pin = 8,  /* GPIO 8 for TX */                                    \
        .rx_pin = 9,  /* GPIO 9 for RX */                                    \
    };                                                                       \
                                                                             \
    DEVICE_DT_DEFINE(DT_NODELABEL(sensor_node_##inst),                       \
                    sensor_node_init,                                        \
                    NULL,                                                    \
                    &sensor_node_data_##inst,                                \
                    &sensor_node_config_##inst,                              \
                    POST_KERNEL,                                             \
                    CONFIG_SENSOR_INIT_PRIORITY,                             \
                    &sensor_node_driver_api);

/* Create an instance of the driver */
SENSOR_NODE_INIT(0);