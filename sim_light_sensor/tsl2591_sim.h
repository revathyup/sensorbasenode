/**
 * @file tsl2591_sim.h
 * @brief Simulated driver for the TSL2591 light sensor
 */

#ifndef TSL2591_SIM_H
#define TSL2591_SIM_H

#include <stdbool.h>
#include <stdint.h>

// TSL2591 gain settings
typedef enum {
    TSL2591_GAIN_1X = 0x00,
    TSL2591_GAIN_25X = 0x10,
    TSL2591_GAIN_428X = 0x20,
    TSL2591_GAIN_9876X = 0x30
} tsl2591_gain_t;

// TSL2591 integration time settings
typedef enum {
    TSL2591_INTEGRATIONTIME_100 = 0x00,
    TSL2591_INTEGRATIONTIME_200 = 0x01,
    TSL2591_INTEGRATIONTIME_300 = 0x02,
    TSL2591_INTEGRATIONTIME_400 = 0x03,
    TSL2591_INTEGRATIONTIME_500 = 0x04,
    TSL2591_INTEGRATIONTIME_600 = 0x05
} tsl2591_integration_time_t;

/**
 * @brief Initialize the simulated TSL2591 sensor
 * 
 * @return true if initialization successful, false otherwise
 */
bool tsl2591_init(void);

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
 * @brief Read the full luminosity (visible + IR) from the sensor
 * 
 * @param channel0 Pointer to store full spectrum reading
 * @param channel1 Pointer to store infrared reading
 * @return true if successful, false otherwise
 */
bool tsl2591_get_full_luminosity(uint16_t *channel0, uint16_t *channel1);

/**
 * @brief Calculate lux based on sensor readings
 * 
 * @param ch0 Full spectrum reading
 * @param ch1 Infrared reading
 * @return float Lux value
 */
float tsl2591_calculate_lux(uint16_t ch0, uint16_t ch1);

/**
 * @brief Simulate changing light levels
 * 
 * @param mode 0=normal, 1=low light, 2=bright light, 3=random
 */
void tsl2591_sim_set_mode(int mode);

#endif /* TSL2591_SIM_H */