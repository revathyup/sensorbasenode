/**
 * @file uart_protocol.h
 * @brief UART protocol implementation for base node
 */

#ifndef UART_PROTOCOL_H
#define UART_PROTOCOL_H

#include <stdbool.h>
#include <stdint.h>
#include "pico/stdlib.h"
#include "hardware/uart.h"
#include "../protocol.h"

/**
 * @brief Initialize UART for protocol communication
 * 
 * @param uart_id UART instance to use (0 or 1)
 * @param baud_rate Baud rate for UART communication
 * @param tx_pin TX pin number
 * @param rx_pin RX pin number
 * @return true if initialization successful, false otherwise
 */
bool uart_protocol_init(uart_inst_t *uart_id, uint baud_rate, uint tx_pin, uint rx_pin);

/**
 * @brief Send a command to the sensor node
 * 
 * @param command Command type
 * @param data Data to send
 * @param length Length of data
 * @return true if sent successfully, false otherwise
 */
bool uart_protocol_send_command(uint8_t command, uint8_t *data, uint8_t length);

/**
 * @brief Send an acknowledgment packet
 * 
 * @param command Command being acknowledged
 * @return true if sent successfully, false otherwise
 */
bool uart_protocol_send_ack(uint8_t command);

/**
 * @brief Send a negative acknowledgment packet
 * 
 * @param command Command being rejected
 * @param error_code Error code
 * @return true if sent successfully, false otherwise
 */
bool uart_protocol_send_nack(uint8_t command, uint8_t error_code);

/**
 * @brief Send a ping command
 * 
 * @return true if sent successfully, false otherwise
 */
bool uart_protocol_send_ping(void);

/**
 * @brief Check if there's any incoming data
 * 
 * @return true if data is available, false otherwise
 */
bool uart_protocol_data_available(void);

/**
 * @brief Receive and parse a packet
 * 
 * @param packet Pointer to store the received packet
 * @return true if packet received successfully, false otherwise
 */
bool uart_protocol_receive_packet(protocol_packet_t *packet);

/**
 * @brief Calculate checksum for a packet
 * 
 * @param packet Protocol packet
 * @return uint8_t Calculated checksum
 */
uint8_t uart_protocol_calculate_checksum(protocol_packet_t *packet);

/**
 * @brief Validate a received packet
 * 
 * @param packet Received packet
 * @return true if packet is valid, false otherwise
 */
bool uart_protocol_validate_packet(protocol_packet_t *packet);

/**
 * @brief Parse light sensor data from a packet
 * 
 * @param packet Protocol packet
 * @param light_data Pointer to store light data
 * @return true if parsing successful, false otherwise
 */
bool uart_protocol_parse_light_data(protocol_packet_t *packet, light_data_t *light_data);

#endif /* UART_PROTOCOL_H */