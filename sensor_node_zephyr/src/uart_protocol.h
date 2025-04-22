/**
 * @file uart_protocol.h
 * @brief UART communication protocol for sensor node
 */

#ifndef UART_PROTOCOL_H
#define UART_PROTOCOL_H

#include <stdbool.h>
#include <zephyr/device.h>
#include "protocol.h"

// Function prototypes
bool uart_protocol_init(const struct device *uart_dev);

/**
 * @brief Send sensor data over UART
 * 
 * @param sensor_type Type of sensor (SENSOR_LIGHT, SENSOR_TEMPERATURE, etc.)
 * @param value Sensor value (scaled by 100 for floating point values)
 * @return true if sent successfully, false otherwise
 */
bool uart_protocol_send_sensor_data(uint8_t sensor_type, int32_t value);

/**
 * @brief Send interrupt notification over UART
 * 
 * @param interrupt_type Type of interrupt (INT_HIGH_TEMPERATURE, etc.)
 * @param value Current value that triggered the interrupt
 * @param threshold Threshold that was crossed
 * @return true if sent successfully, false otherwise
 */
bool uart_protocol_send_interrupt(uint8_t interrupt_type, int32_t value, int32_t threshold);

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

/**
 * @brief Sets the callback function for handling configuration requests
 * 
 * @param callback Function to be called when configuration requests are received
 */
void uart_protocol_set_config_callback(void (*callback)(uint8_t param, int32_t value));

#endif /* UART_PROTOCOL_H */