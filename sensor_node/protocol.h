/**
 * @file protocol.h
 * @brief Communication protocol between sensor node and base node
 */
#ifndef PROTOCOL_H
#define PROTOCOL_H

#include <stdint.h>

// Protocol constants
#define PROTOCOL_START_BYTE 0xAA
#define PROTOCOL_MAX_DATA_LENGTH 32

// Command byte values
#define CMD_PING             0x01
#define CMD_SENSOR_DATA      0x02
#define CMD_CONFIG_THRESHOLD 0x03
#define CMD_ACK              0x04
#define CMD_ERROR            0x05
#define CMD_INTERRUPT        0x06
#define CMD_READ_SENSOR      0x07
#define CMD_RESET            0x08

// Sensor type identifiers
#define SENSOR_LIGHT         0x01
#define SENSOR_TEMPERATURE   0x02
#define SENSOR_SOIL_MOISTURE 0x03
#define SENSOR_ALL           0xFF

// Error codes
#define ERR_INVALID_COMMAND  0x01
#define ERR_INVALID_DATA     0x02
#define ERR_CHECKSUM_FAIL    0x03
#define ERR_SENSOR_FAIL      0x04

// Thresholds for interrupts
#define THRESHOLD_TYPE_MIN   0x01    // Trigger when below threshold
#define THRESHOLD_TYPE_MAX   0x02    // Trigger when above threshold

// Packet structure
typedef struct {
    uint8_t start;           // Start byte (always PROTOCOL_START_BYTE)
    uint8_t command;         // Command byte
    uint8_t data_length;     // Length of data field
    uint8_t data[PROTOCOL_MAX_DATA_LENGTH]; // Data field
    uint8_t checksum;        // XOR of all previous bytes
} __attribute__((packed)) protocol_packet_t;

// Sensor data structure
typedef struct {
    uint8_t sensor_type;     // Type of sensor
    uint16_t value;          // Sensor value
} __attribute__((packed)) sensor_data_t;

// Configuration data structure
typedef struct {
    uint8_t sensor_type;     // Type of sensor
    uint8_t threshold_type;  // Type of threshold (min/max)
    uint16_t threshold;      // Threshold value
} __attribute__((packed)) config_data_t;

// Interrupt data structure
typedef struct {
    uint8_t sensor_type;     // Type of sensor that triggered interrupt
    uint16_t value;          // Current value that triggered interrupt
    uint16_t threshold;      // Threshold that was crossed
    uint8_t threshold_type;  // Type of threshold that was crossed
} __attribute__((packed)) interrupt_data_t;

#endif /* PROTOCOL_H */
