/**
 * @file uart_protocol.c
 * @brief UART protocol implementation for base node
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
 * @brief Send a command to the sensor node
 * 
 * @param command Command type
 * @param data Data to send
 * @param length Length of data
 * @return true if sent successfully, false otherwise
 */
bool uart_protocol_send_command(uint8_t command, uint8_t *data, uint8_t length) {
    protocol_packet_t packet;
    
    // Prepare packet
    packet.start = PROTOCOL_START_BYTE;
    packet.command = command;
    packet.length = length;
    
    // Copy data if provided
    if (data != NULL && length > 0) {
        memcpy(packet.data, data, length);
    }
    
    // Send the packet
    return uart_protocol_send_packet(&packet);
}

/**
 * @brief Send an acknowledgment packet
 * 
 * @param command Command being acknowledged
 * @return true if sent successfully, false otherwise
 */
bool uart_protocol_send_ack(uint8_t command) {
    protocol_packet_t packet;
    
    // Prepare packet
    packet.start = PROTOCOL_START_BYTE;
    packet.command = CMD_ACK;
    packet.length = 1;
    packet.data[0] = command;
    
    // Send the packet
    return uart_protocol_send_packet(&packet);
}

/**
 * @brief Send a negative acknowledgment packet
 * 
 * @param command Command being rejected
 * @param error_code Error code
 * @return true if sent successfully, false otherwise
 */
bool uart_protocol_send_nack(uint8_t command, uint8_t error_code) {
    protocol_packet_t packet;
    
    // Prepare packet
    packet.start = PROTOCOL_START_BYTE;
    packet.command = CMD_NACK;
    packet.length = 2;
    packet.data[0] = command;
    packet.data[1] = error_code;
    
    // Send the packet
    return uart_protocol_send_packet(&packet);
}

/**
 * @brief Send a ping command
 * 
 * @return true if sent successfully, false otherwise
 */
bool uart_protocol_send_ping(void) {
    protocol_packet_t packet;
    
    // Prepare packet
    packet.start = PROTOCOL_START_BYTE;
    packet.command = CMD_PING;
    packet.length = 0;
    
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

/**
 * @brief Parse light sensor data from a packet
 * 
 * @param packet Protocol packet
 * @param light_data Pointer to store light data
 * @return true if parsing successful, false otherwise
 */
bool uart_protocol_parse_light_data(protocol_packet_t *packet, light_data_t *light_data) {
    // Validate packet command and length
    if (packet->command != CMD_SENSOR_DATA || packet->length < 11) {
        return false;
    }
    
    // Check sensor type
    if (packet->data[0] != SENSOR_LIGHT) {
        return false;
    }
    
    // Parse sensor type
    light_data->sensor_type = packet->data[0];
    
    // Parse full spectrum reading
    light_data->full = (packet->data[1] << 8) | packet->data[2];
    
    // Parse IR reading
    light_data->ir = (packet->data[3] << 8) | packet->data[4];
    
    // Parse visible light reading
    light_data->visible = (packet->data[5] << 8) | packet->data[6];
    
    // Parse lux value (4 bytes float)
    float *lux_ptr = (float*)&packet->data[7];
    light_data->lux = *lux_ptr;
    
    return true;
}