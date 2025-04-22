/**
 * @file uart_protocol.c
 * @brief UART protocol implementation for sensor node
 */

#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/uart.h"
#include "uart_protocol.h"

// Static variables
static uart_inst_t *uart_instance;

/**
 * @brief Initialize UART for protocol communication
 * 
 * @param uart_id UART instance to use (0 or 1)
 * @param baud_rate Baud rate for UART communication
 * @param tx_pin TX pin number
 * @param rx_pin RX pin number
 * @return true if initialization successful, false otherwise
 */
bool uart_protocol_init(uart_inst_t *uart_id, uint baud_rate, uint tx_pin, uint rx_pin) {
    uart_instance = uart_id;
    
    // Initialize UART
    uart_init(uart_instance, baud_rate);
    
    // Set the GPIO pin functions
    gpio_set_function(tx_pin, GPIO_FUNC_UART);
    gpio_set_function(rx_pin, GPIO_FUNC_UART);
    
    // Enable FIFO
    uart_set_fifo_enabled(uart_instance, true);
    
    printf("UART protocol initialized: Baud rate %d, TX: GP%d, RX: GP%d\n", 
           baud_rate, tx_pin, rx_pin);
    
    return true;
}

/**
 * @brief Calculate checksum for a packet
 * 
 * @param packet Protocol packet
 * @return uint8_t Calculated checksum
 */
uint8_t uart_protocol_calculate_checksum(protocol_packet_t *packet) {
    uint8_t checksum = 0;
    
    // XOR all bytes in the packet (excluding the checksum field)
    checksum ^= packet->start;
    checksum ^= packet->command;
    checksum ^= packet->length;
    
    // XOR all data bytes
    for (int i = 0; i < packet->length; i++) {
        checksum ^= packet->data[i];
    }
    
    return checksum;
}

/**
 * @brief Validate a received packet
 * 
 * @param packet Received packet
 * @return true if packet is valid, false otherwise
 */
bool uart_protocol_validate_packet(protocol_packet_t *packet) {
    // Check start byte
    if (packet->start != PROTOCOL_START_BYTE) {
        return false;
    }
    
    // Check data length is within bounds
    if (packet->length > PROTOCOL_MAX_DATA_LENGTH) {
        return false;
    }
    
    // Calculate and check checksum
    uint8_t calculated_checksum = uart_protocol_calculate_checksum(packet);
    if (calculated_checksum != packet->checksum) {
        return false;
    }
    
    return true;
}

/**
 * @brief Send a packet over UART
 * 
 * @param packet Protocol packet to send
 * @return true if sent successfully, false otherwise
 */
static bool uart_protocol_send_packet(protocol_packet_t *packet) {
    // Calculate checksum
    packet->checksum = uart_protocol_calculate_checksum(packet);
    
    // Send start byte
    uart_putc(uart_instance, packet->start);
    
    // Send command
    uart_putc(uart_instance, packet->command);
    
    // Send data length
    uart_putc(uart_instance, packet->length);
    
    // Send data payload
    for (int i = 0; i < packet->length; i++) {
        uart_putc(uart_instance, packet->data[i]);
    }
    
    // Send checksum
    uart_putc(uart_instance, packet->checksum);
    
    return true;
}

/**
 * @brief Send light sensor data over UART
 * 
 * @param data Light sensor data structure
 * @return true if sent successfully, false otherwise
 */
bool uart_protocol_send_light_data(light_data_t *data) {
    protocol_packet_t packet;
    
    // Prepare packet
    packet.start = PROTOCOL_START_BYTE;
    packet.command = CMD_SENSOR_DATA;
    
    // Prepare data payload
    packet.data[0] = data->sensor_type;
    
    // Copy full spectrum reading (2 bytes)
    packet.data[1] = (data->full >> 8) & 0xFF;  // High byte
    packet.data[2] = data->full & 0xFF;         // Low byte
    
    // Copy IR reading (2 bytes)
    packet.data[3] = (data->ir >> 8) & 0xFF;    // High byte
    packet.data[4] = data->ir & 0xFF;           // Low byte
    
    // Copy visible reading (2 bytes)
    packet.data[5] = (data->visible >> 8) & 0xFF;  // High byte
    packet.data[6] = data->visible & 0xFF;         // Low byte
    
    // Copy lux value (4 bytes)
    float *lux_ptr = (float*)&packet.data[7];
    *lux_ptr = data->lux;
    
    // Set data length (1 byte sensor type + 6 bytes readings + 4 bytes lux)
    packet.length = 11;
    
    // Send the packet
    return uart_protocol_send_packet(&packet);
}

/**
 * @brief Send alert over UART
 * 
 * @param alert_type Type of alert
 * @param value Value that triggered the alert
 * @param threshold Threshold that was crossed
 * @return true if sent successfully, false otherwise
 */
bool uart_protocol_send_alert(uint8_t alert_type, float value, float threshold) {
    protocol_packet_t packet;
    
    // Prepare packet
    packet.start = PROTOCOL_START_BYTE;
    packet.command = CMD_ALERT;
    
    // Prepare data payload
    packet.data[0] = alert_type;
    
    // Copy value (4 bytes)
    float *value_ptr = (float*)&packet.data[1];
    *value_ptr = value;
    
    // Copy threshold (4 bytes)
    float *threshold_ptr = (float*)&packet.data[5];
    *threshold_ptr = threshold;
    
    // Set data length (1 byte alert type + 4 bytes value + 4 bytes threshold)
    packet.length = 9;
    
    // Send the packet
    return uart_protocol_send_packet(&packet);
}

/**
 * @brief Check if there's any incoming data
 * 
 * @return true if data is available, false otherwise
 */
bool uart_protocol_data_available(void) {
    return uart_is_readable(uart_instance);
}

/**
 * @brief Receive and parse a packet
 * 
 * @param packet Pointer to store the received packet
 * @return true if packet received successfully, false otherwise
 */
bool uart_protocol_receive_packet(protocol_packet_t *packet) {
    // Check if there's data available
    if (!uart_is_readable(uart_instance)) {
        return false;
    }
    
    // Wait for start byte
    while (uart_is_readable(uart_instance)) {
        uint8_t byte = uart_getc(uart_instance);
        if (byte == PROTOCOL_START_BYTE) {
            packet->start = byte;
            break;
        }
    }
    
    // Check if we have more data
    if (!uart_is_readable_within_us(uart_instance, 1000)) {
        return false;
    }
    
    // Read command
    packet->command = uart_getc(uart_instance);
    
    // Read data length
    if (!uart_is_readable_within_us(uart_instance, 1000)) {
        return false;
    }
    packet->length = uart_getc(uart_instance);
    
    // Validate data length
    if (packet->length > PROTOCOL_MAX_DATA_LENGTH) {
        return false;
    }
    
    // Read data payload
    for (int i = 0; i < packet->length; i++) {
        if (!uart_is_readable_within_us(uart_instance, 1000)) {
            return false;
        }
        packet->data[i] = uart_getc(uart_instance);
    }
    
    // Read checksum
    if (!uart_is_readable_within_us(uart_instance, 1000)) {
        return false;
    }
    packet->checksum = uart_getc(uart_instance);
    
    // Validate packet
    if (!uart_protocol_validate_packet(packet)) {
        return false;
    }
    
    return true;
}