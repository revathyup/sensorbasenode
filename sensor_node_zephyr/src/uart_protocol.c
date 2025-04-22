/**
 * @file uart_protocol.c
 * @brief UART communication protocol implementation for sensor node
 */
#include "uart_protocol.h"
#include <zephyr/kernel.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/logging/log.h>
#include <string.h>

LOG_MODULE_REGISTER(uart_proto, CONFIG_LOG_DEFAULT_LEVEL);

// Static variables
static const struct device *uart_device = NULL;
static message_t tx_message;
static message_t rx_message;
static uint8_t rx_buffer[MAX_MESSAGE_SIZE];
static uint8_t rx_pos = 0;
static uint8_t sequence_number = 0;
static void (*config_callback)(uint8_t param, int32_t value) = NULL;

// UART interrupt callback function
static void uart_cb(const struct device *dev, void *user_data);

/**
 * @brief Calculate checksum for a message
 * 
 * @param msg Pointer to message
 * @return uint8_t Calculated checksum
 */
static uint8_t calculate_checksum(const message_t *msg) {
    uint8_t sum = 0;
    const uint8_t *ptr = (const uint8_t *)msg;
    uint8_t length = sizeof(message_header_t) + msg->header.length;
    
    for (uint8_t i = 0; i < length; i++) {
        sum += ptr[i];
    }
    
    return sum;
}

/**
 * @brief Initialize UART protocol
 * 
 * @param uart_dev UART device to use
 * @return true if initialized successfully, false otherwise
 */
bool uart_protocol_init(const struct device *uart_dev) {
    uart_device = uart_dev;
    
    if (!device_is_ready(uart_device)) {
        LOG_ERR("UART device not ready");
        return false;
    }
    
    // Set up UART callback
    uart_irq_callback_set(uart_device, uart_cb);
    
    // Enable UART receive interrupts
    uart_irq_rx_enable(uart_device);
    
    LOG_INF("UART protocol initialized");
    return true;
}

/**
 * @brief Send a message over UART
 * 
 * @param msg Message to send
 * @return true if sent successfully, false otherwise
 */
static bool send_message(const message_t *msg) {
    uint8_t length = sizeof(message_header_t) + msg->header.length + 1; // +1 for checksum
    const uint8_t *data = (const uint8_t *)msg;
    
    for (uint8_t i = 0; i < length; i++) {
        if (uart_poll_out(uart_device, data[i]) < 0) {
            LOG_ERR("Failed to send byte over UART");
            return false;
        }
    }
    
    return true;
}

/**
 * @brief Send sensor data over UART
 * 
 * @param sensor_type Type of sensor (SENSOR_LIGHT, SENSOR_TEMPERATURE, etc.)
 * @param value Sensor value (scaled by 100 for floating point values)
 * @return true if sent successfully, false otherwise
 */
bool uart_protocol_send_sensor_data(uint8_t sensor_type, int32_t value) {
    // Fill header
    tx_message.header.start_byte = PROTOCOL_START_BYTE;
    tx_message.header.message_type = MSG_SENSOR_DATA;
    tx_message.header.length = sizeof(sensor_data_t);
    tx_message.header.sequence = sequence_number++;
    
    // Fill payload
    tx_message.payload.sensor.timestamp = k_uptime_get_32();
    tx_message.payload.sensor.sensor_type = sensor_type;
    tx_message.payload.sensor.value = value;
    
    // Calculate and add checksum
    tx_message.payload.raw[tx_message.header.length] = calculate_checksum(&tx_message);
    
    // Send message
    return send_message(&tx_message);
}

/**
 * @brief Send interrupt notification over UART
 * 
 * @param interrupt_type Type of interrupt (INT_HIGH_TEMPERATURE, etc.)
 * @param value Current value that triggered the interrupt
 * @param threshold Threshold that was crossed
 * @return true if sent successfully, false otherwise
 */
bool uart_protocol_send_interrupt(uint8_t interrupt_type, int32_t value, int32_t threshold) {
    // Fill header
    tx_message.header.start_byte = PROTOCOL_START_BYTE;
    tx_message.header.message_type = MSG_INTERRUPT;
    tx_message.header.length = sizeof(interrupt_data_t);
    tx_message.header.sequence = sequence_number++;
    
    // Fill payload
    tx_message.payload.interrupt.timestamp = k_uptime_get_32();
    tx_message.payload.interrupt.interrupt_type = interrupt_type;
    tx_message.payload.interrupt.value = value;
    tx_message.payload.interrupt.threshold = threshold;
    
    // Calculate and add checksum
    tx_message.payload.raw[tx_message.header.length] = calculate_checksum(&tx_message);
    
    // Send message
    return send_message(&tx_message);
}

/**
 * @brief Send error message over UART
 * 
 * @param error_code Error code to send
 * @param context Context-specific data
 * @return true if sent successfully, false otherwise
 */
bool uart_protocol_send_error(uint8_t error_code, uint8_t context) {
    // Fill header
    tx_message.header.start_byte = PROTOCOL_START_BYTE;
    tx_message.header.message_type = MSG_ERROR;
    tx_message.header.length = sizeof(error_data_t);
    tx_message.header.sequence = sequence_number++;
    
    // Fill payload
    tx_message.payload.error.error_code = error_code;
    tx_message.payload.error.source = 0; // 0 = sensor node
    tx_message.payload.error.context = context;
    
    // Calculate and add checksum
    tx_message.payload.raw[tx_message.header.length] = calculate_checksum(&tx_message);
    
    // Send message
    return send_message(&tx_message);
}

/**
 * @brief Send configuration response over UART
 * 
 * @param param Parameter that was configured
 * @param value Value that was set
 * @return true if sent successfully, false otherwise
 */
static bool send_config_response(uint8_t param, int32_t value) {
    // Fill header
    tx_message.header.start_byte = PROTOCOL_START_BYTE;
    tx_message.header.message_type = MSG_CONFIG_RESPONSE;
    tx_message.header.length = sizeof(config_data_t);
    tx_message.header.sequence = sequence_number++;
    
    // Fill payload
    tx_message.payload.config.param = param;
    tx_message.payload.config.value = value;
    
    // Calculate and add checksum
    tx_message.payload.raw[tx_message.header.length] = calculate_checksum(&tx_message);
    
    // Send message
    return send_message(&tx_message);
}

/**
 * @brief Send acknowledgment message over UART
 * 
 * @param ack_sequence Sequence number to acknowledge
 * @return true if sent successfully, false otherwise
 */
static bool send_ack(uint8_t ack_sequence) {
    // Fill header
    tx_message.header.start_byte = PROTOCOL_START_BYTE;
    tx_message.header.message_type = MSG_ACK;
    tx_message.header.length = 1; // Just one byte for the sequence number
    tx_message.header.sequence = sequence_number++;
    
    // Fill payload
    tx_message.payload.raw[0] = ack_sequence;
    
    // Calculate and add checksum
    tx_message.payload.raw[tx_message.header.length] = calculate_checksum(&tx_message);
    
    // Send message
    return send_message(&tx_message);
}

/**
 * @brief Process a received message
 * 
 * @param msg Pointer to the received message
 */
static void process_message(const message_t *msg) {
    uint8_t calculated_checksum = calculate_checksum(msg);
    uint8_t received_checksum = msg->payload.raw[msg->header.length];
    
    // Verify checksum
    if (calculated_checksum != received_checksum) {
        LOG_ERR("Invalid checksum: calculated=0x%02X, received=0x%02X", 
                calculated_checksum, received_checksum);
        return;
    }
    
    // Process based on message type
    switch (msg->header.message_type) {
        case MSG_CONFIG_REQUEST:
            if (config_callback) {
                config_callback(msg->payload.config.param, msg->payload.config.value);
                send_config_response(msg->payload.config.param, msg->payload.config.value);
            }
            break;
            
        case MSG_ACK:
            // Process acknowledgment (could be used for reliability)
            break;
            
        case MSG_ERROR:
            LOG_WRN("Received error from base: code=0x%02X, context=0x%02X",
                    msg->payload.error.error_code, msg->payload.error.context);
            break;
            
        default:
            LOG_WRN("Unhandled message type: 0x%02X", msg->header.message_type);
            break;
    }
    
    // Send acknowledgment for all received messages except ACKs
    if (msg->header.message_type != MSG_ACK) {
        send_ack(msg->header.sequence);
    }
}

/**
 * @brief Process a received byte in the message state machine
 * 
 * @param byte Received byte
 * @return true if a complete message was processed, false otherwise
 */
static bool process_byte(uint8_t byte) {
    static enum {
        WAIT_START,
        RX_HEADER,
        RX_PAYLOAD,
        RX_CHECKSUM
    } state = WAIT_START;
    
    static uint8_t bytes_needed = 0;
    
    switch (state) {
        case WAIT_START:
            if (byte == PROTOCOL_START_BYTE) {
                // Reset buffer and start receiving message
                rx_pos = 0;
                rx_buffer[rx_pos++] = byte;
                state = RX_HEADER;
                bytes_needed = sizeof(message_header_t) - 1; // -1 because we already got the start byte
            }
            break;
            
        case RX_HEADER:
            rx_buffer[rx_pos++] = byte;
            bytes_needed--;
            
            if (bytes_needed == 0) {
                // Header complete, extract message length
                memcpy(&rx_message, rx_buffer, sizeof(message_header_t));
                
                // Validate message length
                if (rx_message.header.length > (MAX_MESSAGE_SIZE - sizeof(message_header_t) - 1)) {
                    LOG_ERR("Invalid message length: %d", rx_message.header.length);
                    state = WAIT_START;
                    return false;
                }
                
                // Start receiving payload
                state = RX_PAYLOAD;
                bytes_needed = rx_message.header.length;
                
                // If no payload, go directly to checksum
                if (bytes_needed == 0) {
                    state = RX_CHECKSUM;
                    bytes_needed = 1;
                }
            }
            break;
            
        case RX_PAYLOAD:
            rx_buffer[rx_pos++] = byte;
            bytes_needed--;
            
            if (bytes_needed == 0) {
                // Payload complete, start receiving checksum
                state = RX_CHECKSUM;
                bytes_needed = 1;
            }
            break;
            
        case RX_CHECKSUM:
            rx_buffer[rx_pos++] = byte;
            
            // Message complete, copy to message structure
            memcpy(&rx_message, rx_buffer, sizeof(message_header_t) + rx_message.header.length);
            rx_message.payload.raw[rx_message.header.length] = byte; // Checksum
            
            // Process the message
            process_message(&rx_message);
            
            // Reset for next message
            state = WAIT_START;
            return true;
    }
    
    return false;
}

/**
 * @brief Process received messages
 * 
 * Should be called regularly to process any incoming messages
 * 
 * @return true if a message was processed, false otherwise
 */
bool uart_protocol_process_messages(void) {
    // Not needed with interrupt-driven implementation
    // The uart_cb function handles all received data
    return false;
}

/**
 * @brief UART interrupt callback function
 * 
 * @param dev UART device
 * @param user_data User data pointer (not used)
 */
static void uart_cb(const struct device *dev, void *user_data) {
    if (!uart_irq_update(dev)) {
        return;
    }
    
    if (uart_irq_rx_ready(dev)) {
        uint8_t byte;
        
        while (uart_fifo_read(dev, &byte, 1) == 1) {
            process_byte(byte);
        }
    }
}

/**
 * @brief Sets the callback function for handling configuration requests
 * 
 * @param callback Function to be called when configuration requests are received
 */
void uart_protocol_set_config_callback(void (*callback)(uint8_t param, int32_t value)) {
    config_callback = callback;
}