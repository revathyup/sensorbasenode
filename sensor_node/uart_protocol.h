/**
 * @file uart_protocol.h
 * @brief UART protocol implementation for sensor node
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
 * @brief Send light sensor data over UART
 * 
 * @param data Light sensor data structure
 * @return true if sent successfully, false otherwise
 */
bool uart_protocol_send_light_data(light_data_t *data);

/**
 * @brief Send alert over UART
 * 
 * @param alert_type Type of alert
 * @param value Value that triggered the alert
 * @param threshold Threshold that was crossed
 * @return true if sent successfully, false otherwise
 */
bool uart_protocol_send_alert(uint8_t alert_type, float value, float threshold);

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

#endif /* UART_PROTOCOL_H */
