/**
 * @file adc.h
 * @brief Mock Pico SDK ADC hardware header for demonstration
 */

#ifndef HARDWARE_ADC_H
#define HARDWARE_ADC_H

#include <stdint.h>
#include <stdbool.h>

// ADC functions
static inline void adc_init(void) {}
static inline void adc_gpio_init(uint gpio) {}
static inline void adc_select_input(uint input) {}
static inline uint16_t adc_read(void) { return 0; }
static inline void adc_set_temp_sensor_enabled(bool enabled) {}
static inline float adc_read_temp_sensor(void) { return 25.0f; }

// ADC constants
#define ADC_TEMP_SENSOR_INPUT 4
#define ADC_VREF 3.3f

#endif /* HARDWARE_ADC_H */