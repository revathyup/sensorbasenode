/**
 * @file protocol.h
 * @brief Communication protocol definition for sensor and base nodes
 */

#ifndef PROTOCOL_H
#define PROTOCOL_H

#include <stdint.h>

// Protocol constants
#define PROTOCOL_START_BYTE       0xAA
#define PROTOCOL_MAX_DATA_LENGTH  32

// Command types
#define CMD_SENSOR_DATA           0x01
#define CMD_SENSOR_CONFIG         0x02
#define CMD_ALERT                 0x03
#define CMD_ACK                   0x04
#define CMD_NACK                  0x05
#define CMD_PING                  0x06

// Sensor types
#define SENSOR_LIGHT              0x01
#define SENSOR_TEMPERATURE        0x02
#define SENSOR_SOIL_MOISTURE      0x03

// Alert types
#define ALERT_LIGHT_LOW           0x01
#define ALERT_LIGHT_HIGH          0x02
#define ALERT_TEMP_LOW            0x03
#define ALERT_TEMP_HIGH           0x04
#define ALERT_SOIL_DRY            0x05
#define ALERT_SOIL_WET            0x06

/**
 * @brief Protocol packet structure
 */
typedef struct {
    uint8_t start;          // Start byte (0xAA)
    uint8_t command;        // Command type
    uint8_t length;         // Data length
    uint8_t data[PROTOCOL_MAX_DATA_LENGTH];  // Data payload
    uint8_t checksum;       // Checksum (XOR of all previous bytes)
} protocol_packet_t;

/**
 * @brief Light sensor data structure
 */
typedef struct {
    uint8_t sensor_type;    // Always SENSOR_LIGHT
    uint16_t full;          // Full spectrum reading
    uint16_t ir;            // Infrared reading
    uint16_t visible;       // Visible light reading
    float lux;              // Calculated lux value
} light_data_t;

#endif /* PROTOCOL_H */