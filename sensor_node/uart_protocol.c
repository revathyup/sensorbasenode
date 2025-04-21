/**
 * @file uart_protocol.c
 * @brief UART communication protocol implementation for sensor node
 */

#include "uart_protocol.h"
#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/uart.h"
#include "hardware/irq.h"
#include "sensors.h"

// Static variables
static uint8_t rx_buffer[MAX_MESSAGE_SIZE];
static uint32_t rx_buffer_index = 0;
static uint8_t sequence_number = 0;
static absolute_time_t last_valid_rx = {0};

// Forward declarations
static void process_config_request(const message_t *msg);
static void uart_rx_callback(void);
static void send_message(message_t *msg);
static uint8_t calculate_checksum(const message_t *msg);
static bool validate_message(const message_t *msg);
static void send_ack(uint8_t seq);

/**
 * @brief Initialize UART protocol
 */
bool uart_protocol_init(void) {
    // Initialize UART with specified parameters
    uart_init(UART_ID, UART_BAUD_RATE);
    
    // Configure UART pins
    gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);
    
    // Set up RX interrupt
    uart_set_hw_flow(UART_ID, false, false);
    uart_set_format(UART_ID, 8, 1, UART_PARITY_NONE);
    uart_set_fifo_enabled(UART_ID, true);
    
    // Set up interrupt handler
    irq_set_exclusive_handler(UART_ID == uart0 ? UART0_IRQ : UART1_IRQ, uart_rx_callback);
    irq_set_enabled(UART_ID == uart0 ? UART0_IRQ : UART1_IRQ, true);
    
    // Enable RX interrupt
    uart_set_irq_enables(UART_ID, true, false);
    
    printf("UART protocol initialized\n");
    return true;
}

/**
 * @brief UART receive interrupt handler
 */
static void uart_rx_callback(void) {
    // Check if interrupt is for receive
    while (uart_is_readable(UART_ID)) {
        uint8_t ch = uart_getc(UART_ID);
        
        // Check for start of message
        if (rx_buffer_index == 0 && ch == PROTOCOL_START_BYTE) {
            rx_buffer[rx_buffer_index++] = ch;
        }
        // Continue filling buffer if already started
        else if (rx_buffer_index > 0) {
            // Prevent buffer overflow
            if (rx_buffer_index < MAX_MESSAGE_SIZE) {
                rx_buffer[rx_buffer_index++] = ch;
            } else {
                // Buffer overflow, reset
                rx_buffer_index = 0;
            }
        }
    }
    
    // Record time of latest activity
    last_valid_rx = get_absolute_time();
}

/**
 * @brief Process received messages
 */
bool uart_protocol_process_messages(void) {
    // Check if we have enough bytes for at least a header
    if (rx_buffer_index < sizeof(message_header_t)) {
        return false;
    }
    
    // Try to interpret as a message
    message_t *msg = (message_t *)rx_buffer;
    
    // Check if we have the complete message
    if (rx_buffer_index < sizeof(message_header_t) + msg->header.length + 1) {
        // Not enough data yet
        return false;
    }
    
    // Validate message
    if (!validate_message(msg)) {
        // Invalid message, reset buffer
        rx_buffer_index = 0;
        return false;
    }
    
    // Process message based on type
    switch (msg->header.message_type) {
        case MSG_CONFIG_REQUEST:
            process_config_request(msg);
            break;
            
        case MSG_ACK:
            // Nothing to do for ACKs
            break;
            
        default:
            // Unknown message type
            uart_protocol_send_error(ERR_UNKNOWN, msg->header.message_type);
            break;
    }
    
    // Reset buffer for next message
    rx_buffer_index = 0;
    return true;
}

/**
 * @brief Process configuration request
 */
static void process_config_request(const message_t *msg) {
    // Get current configuration
    sensor_config_t config;
    if (!sensors_get_config(&config)) {
        uart_protocol_send_error(ERR_UNKNOWN, 0);
        return;
    }
    
    // Update configuration based on request
    config_data_t *config_data = (config_data_t *)&msg->payload;
    bool config_updated = true;
    
    switch (config_data->param) {
        case CONFIG_TEMP_HIGH_THRESHOLD:
            config.temp_high_threshold = (float)config_data->value / 100.0f;
            break;
            
        case CONFIG_SOIL_LOW_THRESHOLD:
            config.soil_low_threshold = (float)config_data->value / 100.0f;
            break;
            
        case CONFIG_LIGHT_LOW_THRESHOLD:
            config.light_low_threshold = (float)config_data->value / 100.0f;
            break;
            
        case CONFIG_LIGHT_HIGH_THRESHOLD:
            config.light_high_threshold = (float)config_data->value / 100.0f;
            break;
            
        case CONFIG_SAMPLING_RATE:
            config.sampling_rate_ms = config_data->value;
            break;
            
        case CONFIG_INT_ENABLE:
            // Interpret value as bit flags:
            // bit 0: temperature high
            // bit 1: soil low
            // bit 2: light low
            // bit 3: light high
            config.int_enable_temp_high = (config_data->value & 0x01) != 0;
            config.int_enable_soil_low = (config_data->value & 0x02) != 0;
            config.int_enable_light_low = (config_data->value & 0x04) != 0;
            config.int_enable_light_high = (config_data->value & 0x08) != 0;
            break;
            
        default:
            // Unknown parameter
            config_updated = false;
            uart_protocol_send_error(ERR_INVALID_CONFIG, config_data->param);
            break;
    }
    
    // Apply configuration if updated
    if (config_updated) {
        if (!sensors_set_config(&config)) {
            uart_protocol_send_error(ERR_INVALID_CONFIG, 0);
            return;
        }
        
        // Send ACK
        send_ack(msg->header.sequence);
        
        // Send configuration response
        message_t response;
        memset(&response, 0, sizeof(response));
        
        response.header.start_byte = PROTOCOL_START_BYTE;
        response.header.message_type = MSG_CONFIG_RESPONSE;
        response.header.length = sizeof(config_data_t);
        response.header.sequence = sequence_number++;
        
        response.payload.config.param = config_data->param;
        response.payload.config.value = config_data->value;
        
        response.checksum = calculate_checksum(&response);
        send_message(&response);
    }
}

/**
 * @brief Send sensor data over UART
 */
bool uart_protocol_send_sensor_data(const sensor_data_t *data) {
    if (!data) return false;
    
    // Send each valid sensor reading as a separate message
    bool success = true;
    
    if (data->valid_light) {
        message_t msg;
        memset(&msg, 0, sizeof(msg));
        
        msg.header.start_byte = PROTOCOL_START_BYTE;
        msg.header.message_type = MSG_SENSOR_DATA;
        msg.header.length = sizeof(sensor_data_t);
        msg.header.sequence = sequence_number++;
        
        msg.payload.sensor.timestamp = to_ms_since_boot(get_absolute_time());
        msg.payload.sensor.sensor_type = SENSOR_LIGHT;
        msg.payload.sensor.value = (int32_t)(data->light * 100.0f); // Scale by 100 for 2 decimal places
        
        msg.checksum = calculate_checksum(&msg);
        send_message(&msg);
    }
    
    if (data->valid_temperature) {
        message_t msg;
        memset(&msg, 0, sizeof(msg));
        
        msg.header.start_byte = PROTOCOL_START_BYTE;
        msg.header.message_type = MSG_SENSOR_DATA;
        msg.header.length = sizeof(sensor_data_t);
        msg.header.sequence = sequence_number++;
        
        msg.payload.sensor.timestamp = to_ms_since_boot(get_absolute_time());
        msg.payload.sensor.sensor_type = SENSOR_TEMPERATURE;
        msg.payload.sensor.value = (int32_t)(data->temperature * 100.0f); // Scale by 100 for 2 decimal places
        
        msg.checksum = calculate_checksum(&msg);
        send_message(&msg);
    }
    
    if (data->valid_soil) {
        message_t msg;
        memset(&msg, 0, sizeof(msg));
        
        msg.header.start_byte = PROTOCOL_START_BYTE;
        msg.header.message_type = MSG_SENSOR_DATA;
        msg.header.length = sizeof(sensor_data_t);
        msg.header.sequence = sequence_number++;
        
        msg.payload.sensor.timestamp = to_ms_since_boot(get_absolute_time());
        msg.payload.sensor.sensor_type = SENSOR_SOIL_MOISTURE;
        msg.payload.sensor.value = (int32_t)(data->soil_moisture * 100.0f); // Scale by 100 for 2 decimal places
        
        msg.checksum = calculate_checksum(&msg);
        send_message(&msg);
    }
    
    return success;
}

/**
 * @brief Send interrupt notification over UART
 */
bool uart_protocol_send_interrupt(const sensor_data_t *data, const interrupt_status_t *status) {
    if (!data || !status) return false;
    
    bool success = true;
    
    // Send interrupt message for each active interrupt
    if (status->temp_high && data->valid_temperature) {
        message_t msg;
        memset(&msg, 0, sizeof(msg));
        
        msg.header.start_byte = PROTOCOL_START_BYTE;
        msg.header.message_type = MSG_INTERRUPT;
        msg.header.length = sizeof(interrupt_data_t);
        msg.header.sequence = sequence_number++;
        
        sensor_config_t config;
        sensors_get_config(&config);
        
        msg.payload.interrupt.timestamp = to_ms_since_boot(get_absolute_time());
        msg.payload.interrupt.interrupt_type = INT_HIGH_TEMPERATURE;
        msg.payload.interrupt.value = (int32_t)(data->temperature * 100.0f);
        msg.payload.interrupt.threshold = (int32_t)(config.temp_high_threshold * 100.0f);
        
        msg.checksum = calculate_checksum(&msg);
        send_message(&msg);
    }
    
    if (status->soil_low && data->valid_soil) {
        message_t msg;
        memset(&msg, 0, sizeof(msg));
        
        msg.header.start_byte = PROTOCOL_START_BYTE;
        msg.header.message_type = MSG_INTERRUPT;
        msg.header.length = sizeof(interrupt_data_t);
        msg.header.sequence = sequence_number++;
        
        sensor_config_t config;
        sensors_get_config(&config);
        
        msg.payload.interrupt.timestamp = to_ms_since_boot(get_absolute_time());
        msg.payload.interrupt.interrupt_type = INT_LOW_SOIL_MOISTURE;
        msg.payload.interrupt.value = (int32_t)(data->soil_moisture * 100.0f);
        msg.payload.interrupt.threshold = (int32_t)(config.soil_low_threshold * 100.0f);
        
        msg.checksum = calculate_checksum(&msg);
        send_message(&msg);
    }
    
    if (status->light_low && data->valid_light) {
        message_t msg;
        memset(&msg, 0, sizeof(msg));
        
        msg.header.start_byte = PROTOCOL_START_BYTE;
        msg.header.message_type = MSG_INTERRUPT;
        msg.header.length = sizeof(interrupt_data_t);
        msg.header.sequence = sequence_number++;
        
        sensor_config_t config;
        sensors_get_config(&config);
        
        msg.payload.interrupt.timestamp = to_ms_since_boot(get_absolute_time());
        msg.payload.interrupt.interrupt_type = INT_LOW_LIGHT;
        msg.payload.interrupt.value = (int32_t)(data->light * 100.0f);
        msg.payload.interrupt.threshold = (int32_t)(config.light_low_threshold * 100.0f);
        
        msg.checksum = calculate_checksum(&msg);
        send_message(&msg);
    }
    
    if (status->light_high && data->valid_light) {
        message_t msg;
        memset(&msg, 0, sizeof(msg));
        
        msg.header.start_byte = PROTOCOL_START_BYTE;
        msg.header.message_type = MSG_INTERRUPT;
        msg.header.length = sizeof(interrupt_data_t);
        msg.header.sequence = sequence_number++;
        
        sensor_config_t config;
        sensors_get_config(&config);
        
        msg.payload.interrupt.timestamp = to_ms_since_boot(get_absolute_time());
        msg.payload.interrupt.interrupt_type = INT_HIGH_LIGHT;
        msg.payload.interrupt.value = (int32_t)(data->light * 100.0f);
        msg.payload.interrupt.threshold = (int32_t)(config.light_high_threshold * 100.0f);
        
        msg.checksum = calculate_checksum(&msg);
        send_message(&msg);
    }
    
    return success;
}

/**
 * @brief Send error message over UART
 */
bool uart_protocol_send_error(uint8_t error_code, uint8_t context) {
    message_t msg;
    memset(&msg, 0, sizeof(msg));
    
    msg.header.start_byte = PROTOCOL_START_BYTE;
    msg.header.message_type = MSG_ERROR;
    msg.header.length = sizeof(error_data_t);
    msg.header.sequence = sequence_number++;
    
    msg.payload.error.error_code = error_code;
    msg.payload.error.source = 0; // Sensor node
    msg.payload.error.context = context;
    
    msg.checksum = calculate_checksum(&msg);
    send_message(&msg);
    
    return true;
}

/**
 * @brief Send ACK message
 */
static void send_ack(uint8_t seq) {
    message_t msg;
    memset(&msg, 0, sizeof(msg));
    
    msg.header.start_byte = PROTOCOL_START_BYTE;
    msg.header.message_type = MSG_ACK;
    msg.header.length = 1; // Just sequence number
    msg.header.sequence = sequence_number++;
    
    msg.payload.raw[0] = seq; // ACK for which sequence number
    
    msg.checksum = calculate_checksum(&msg);
    send_message(&msg);
}

/**
 * @brief Calculate checksum for a message
 */
static uint8_t calculate_checksum(const message_t *msg) {
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
static bool validate_message(const message_t *msg) {
    if (!msg) return false;
    
    // Check start byte
    if (msg->header.start_byte != PROTOCOL_START_BYTE) {
        return false;
    }
    
    // Check length is reasonable
    if (msg->header.length > 32) {
        return false;
    }
    
    // Calculate and verify checksum
    uint8_t calculated_checksum = calculate_checksum(msg);
    uint8_t received_checksum = msg->payload.raw[msg->header.length]; // Checksum is after payload
    
    return calculated_checksum == received_checksum;
}

/**
 * @brief Send a message over UART
 */
static void send_message(message_t *msg) {
    if (!msg) return;
    
    // Calculate message size
    size_t msg_size = sizeof(message_header_t) + msg->header.length + 1; // +1 for checksum
    
    // Send the message
    uart_write_blocking(UART_ID, (const uint8_t *)msg, msg_size);
}
