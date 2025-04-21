/**
 * @file uart_protocol.h
 * @brief UART communication protocol for sensor node
 */

#ifndef UART_PROTOCOL_H
#define UART_PROTOCOL_H

#include <stdbool.h>
#include "protocol.h"
#include "sensors.h"

// UART interface
#define UART_ID uart0
#define UART_TX_PIN 16
#define UART_RX_PIN 17
#define UART_BAUD_RATE 115200

/**
 * @brief Initialize UART protocol
 * 
 * @return true if initialized successfully, false otherwise
 */
bool uart_protocol_init(void);

/**
 * @brief Send sensor data over UART
 * 
 * @param data Sensor data to send
 * @return true if sent successfully, false otherwise
 */
bool uart_protocol_send_sensor_data(const sensor_data_t *data);

/**
 * @brief Send interrupt notification over UART
 * 
 * @param data Sensor data that triggered the interrupt
 * @param status Interrupt status
 * @return true if sent successfully, false otherwise
 */
bool uart_protocol_send_interrupt(const sensor_data_t *data, const interrupt_status_t *status);

/**
 * @brief Send error message over UART
 * 
 * @param error_code Error code to send
 * @param context Context-specific data
 * @return true if sent successfully, false otherwise
 */
bool uart_protocol_send_error(uint8_t error_code, uint8_t context);

/**
 * @brief Process received messages
 * 
 * Should be called regularly to process any incoming messages
 * 
 * @return true if a message was processed, false otherwise
 */
bool uart_protocol_process_messages(void);

#endif /* UART_PROTOCOL_H */
