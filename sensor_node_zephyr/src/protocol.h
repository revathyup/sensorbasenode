/**
 * @file protocol.h
 * @brief Common protocol definitions for communication between sensor node and base node
 */

#ifndef PROTOCOL_H
#define PROTOCOL_H

#include <stdint.h>

/**
 * @brief Message types for communication protocol
 */
enum message_type {
    MSG_SENSOR_DATA = 0x01,      // Regular sensor data message
    MSG_CONFIG_REQUEST = 0x02,   // Configuration request from base to sensor
    MSG_CONFIG_RESPONSE = 0x03,  // Configuration response from sensor to base
    MSG_INTERRUPT = 0x04,        // Interrupt notification from sensor to base
    MSG_ACK = 0x05,              // Acknowledgment message
    MSG_ERROR = 0xFF,            // Error message
};

/**
 * @brief Sensor types
 */
enum sensor_type {
    SENSOR_LIGHT = 0x01,
    SENSOR_TEMPERATURE = 0x02,
    SENSOR_SOIL_MOISTURE = 0x03,
};

/**
 * @brief Interrupt types
 */
enum interrupt_type {
    INT_HIGH_TEMPERATURE = 0x01,
    INT_LOW_SOIL_MOISTURE = 0x02,
    INT_LOW_LIGHT = 0x03,
    INT_HIGH_LIGHT = 0x04,
};

/**
 * @brief Error codes
 */
enum error_code {
    ERR_SENSOR_FAIL = 0x01,
    ERR_COMMUNICATION = 0x02,
    ERR_INVALID_CONFIG = 0x03,
    ERR_UNKNOWN = 0xFF,
};

/**
 * @brief Configuration parameters
 */
enum config_param {
    CONFIG_TEMP_HIGH_THRESHOLD = 0x01,   // High temperature threshold
    CONFIG_SOIL_LOW_THRESHOLD = 0x02,    // Low soil moisture threshold
    CONFIG_LIGHT_LOW_THRESHOLD = 0x03,   // Low light threshold
    CONFIG_LIGHT_HIGH_THRESHOLD = 0x04,  // High light threshold
    CONFIG_SAMPLING_RATE = 0x05,         // Sampling rate in milliseconds
    CONFIG_INT_ENABLE = 0x06,            // Interrupt enable flags
};

/**
 * @brief Message header structure
 */
typedef struct __attribute__((packed)) {
    uint8_t start_byte;          // Always 0xAA
    uint8_t message_type;        // Type of message (enum message_type)
    uint8_t length;              // Length of payload
    uint8_t sequence;            // Sequence number
} message_header_t;

/**
 * @brief Sensor data payload structure
 */
typedef struct __attribute__((packed)) {
    uint32_t timestamp;          // Timestamp in milliseconds
    uint8_t sensor_type;         // Type of sensor (enum sensor_type)
    int32_t value;               // Sensor value (scaled by 100 for floating point)
} sensor_data_t;

/**
 * @brief Configuration request/response payload structure
 */
typedef struct __attribute__((packed)) {
    uint8_t param;               // Parameter to configure (enum config_param)
    int32_t value;               // Parameter value
} config_data_t;

/**
 * @brief Interrupt notification payload structure
 */
typedef struct __attribute__((packed)) {
    uint32_t timestamp;          // Timestamp in milliseconds
    uint8_t interrupt_type;      // Type of interrupt (enum interrupt_type)
    int32_t value;               // Value that triggered the interrupt
    int32_t threshold;           // Threshold that was crossed
} interrupt_data_t;

/**
 * @brief Error message payload structure
 */
typedef struct __attribute__((packed)) {
    uint8_t error_code;          // Error code (enum error_code)
    uint8_t source;              // Source of error (0 = sensor node, 1 = base node)
    uint8_t context;             // Context-specific data
} error_data_t;

/**
 * @brief Complete message structure
 */
typedef struct __attribute__((packed)) {
    message_header_t header;     // Message header
    union {
        sensor_data_t sensor;
        config_data_t config;
        interrupt_data_t interrupt;
        error_data_t error;
        uint8_t raw[32];         // Raw payload data
    } payload;
    uint8_t checksum;            // Sum of all bytes in header and payload
} message_t;

#define PROTOCOL_START_BYTE 0xAA
#define MAX_MESSAGE_SIZE 40      // Maximum message size in bytes

#endif /* PROTOCOL_H */