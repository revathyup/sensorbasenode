/**
 * @file tsl2591.h
 * @brief Driver for the TSL2591 light sensor
 */

#ifndef TSL2591_H
#define TSL2591_H

#include <stdbool.h>
#include <stdint.h>
#include "hardware/i2c.h"

// TSL2591 I2C address
#define TSL2591_I2C_ADDR 0x29

// Command register constants
#define TSL2591_COMMAND_BIT      0xA0
#define TSL2591_CLEAR_INT        0xE6
#define TSL2591_REGISTER_ENABLE  0x00
#define TSL2591_REGISTER_CONTROL 0x01
#define TSL2591_REGISTER_ID      0x12
#define TSL2591_REGISTER_STATUS  0x13
#define TSL2591_REGISTER_CHAN0_L 0x14
#define TSL2591_REGISTER_CHAN0_H 0x15
#define TSL2591_REGISTER_CHAN1_L 0x16
#define TSL2591_REGISTER_CHAN1_H 0x17

// Enable register constants
#define TSL2591_ENABLE_POWERON   0x01
#define TSL2591_ENABLE_AEN       0x02
#define TSL2591_ENABLE_AIEN      0x10
#define TSL2591_ENABLE_NPIEN     0x80

// Control register constants
#define TSL2591_GAIN_LOW         0x00  // Low gain (1x)
#define TSL2591_GAIN_MED         0x10  // Medium gain (25x)
#define TSL2591_GAIN_HIGH        0x20  // High gain (428x)
#define TSL2591_GAIN_MAX         0x30  // Maximum gain (9876x)

#define TSL2591_INTEGRATIONTIME_100MS  0x00
#define TSL2591_INTEGRATIONTIME_200MS  0x01
#define TSL2591_INTEGRATIONTIME_300MS  0x02
#define TSL2591_INTEGRATIONTIME_400MS  0x03
#define TSL2591_INTEGRATIONTIME_500MS  0x04
#define TSL2591_INTEGRATIONTIME_600MS  0x05

// TSL2591 gain settings
typedef enum {
    TSL2591_GAIN_1X = TSL2591_GAIN_LOW,
    TSL2591_GAIN_25X = TSL2591_GAIN_MED,
    TSL2591_GAIN_428X = TSL2591_GAIN_HIGH,
    TSL2591_GAIN_9876X = TSL2591_GAIN_MAX
} tsl2591_gain_t;

// TSL2591 integration time settings
typedef enum {
    TSL2591_INTEGRATIONTIME_100 = TSL2591_INTEGRATIONTIME_100MS,
    TSL2591_INTEGRATIONTIME_200 = TSL2591_INTEGRATIONTIME_200MS,
    TSL2591_INTEGRATIONTIME_300 = TSL2591_INTEGRATIONTIME_300MS,
    TSL2591_INTEGRATIONTIME_400 = TSL2591_INTEGRATIONTIME_400MS,
    TSL2591_INTEGRATIONTIME_500 = TSL2591_INTEGRATIONTIME_500MS,
    TSL2591_INTEGRATIONTIME_600 = TSL2591_INTEGRATIONTIME_600MS
} tsl2591_integration_time_t;

/**
 * @brief Initialize the TSL2591 sensor
 * 
 * @param i2c I2C instance to use
 * @return true if initialization successful, false otherwise
 */
bool tsl2591_init(i2c_inst_t *i2c);

/**
 * @brief Set the gain for the TSL2591 sensor
 * 
 * @param gain Gain setting
 * @return true if successful, false otherwise
 */
bool tsl2591_set_gain(tsl2591_gain_t gain);

/**
 * @brief Set the integration time for the TSL2591 sensor
 * 
 * @param integration_time Integration time setting
 * @return true if successful, false otherwise
 */
bool tsl2591_set_integration_time(tsl2591_integration_time_t integration_time);

/**
 * @brief Enable the TSL2591 sensor
 * 
 * @return true if successful, false otherwise
 */
bool tsl2591_enable(void);

/**
 * @brief Disable the TSL2591 sensor
 * 
 * @return true if successful, false otherwise
 */
bool tsl2591_disable(void);

/**
 * @brief Read the full luminosity (visible + IR) from the sensor
 * 
 * @param channel0 Pointer to store full spectrum reading
 * @param channel1 Pointer to store infrared reading
 * @return true if successful, false otherwise
 */
bool tsl2591_get_full_luminosity(uint16_t *channel0, uint16_t *channel1);

/**
 * @brief Get visible light reading
 * 
 * @return uint16_t Visible light reading (full - IR)
 */
uint16_t tsl2591_get_visible_light(void);

/**
 * @brief Calculate lux based on sensor readings
 * 
 * @param ch0 Full spectrum reading
 * @param ch1 Infrared reading
 * @return float Lux value
 */
float tsl2591_calculate_lux(uint16_t ch0, uint16_t ch1);

#endif /* TSL2591_H */