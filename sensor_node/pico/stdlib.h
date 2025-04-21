/**
 * @file stdlib.h
 * @brief Mock Pico SDK stdlib header for demonstration purposes
 */

#ifndef PICO_STDLIB_H
#define PICO_STDLIB_H

#include <stdint.h>
#include <stdbool.h>

// Mock time structure
typedef struct {
    uint64_t _private_us_since_boot;
} absolute_time_t;

// Function declarations for time
static inline uint32_t to_ms_since_boot(absolute_time_t t) { return 0; }
static inline absolute_time_t get_absolute_time(void) { return (absolute_time_t){0}; }

// GPIO functions
static inline void gpio_init(uint gpio) {}
static inline void gpio_set_dir(uint gpio, bool out) {}
static inline void gpio_put(uint gpio, bool value) {}
static inline bool gpio_get(uint gpio) { return false; }
static inline void gpio_set_function(uint gpio, uint function) {}

// Sleep functions
static inline void sleep_ms(uint32_t ms) {}
static inline void sleep_us(uint64_t us) {}

// Standard initialization
static inline void stdio_init_all(void) {}

// GPIO function constants
#define GPIO_FUNC_XIP 0
#define GPIO_FUNC_SPI 1
#define GPIO_FUNC_UART 2
#define GPIO_FUNC_I2C 3
#define GPIO_FUNC_PWM 4
#define GPIO_FUNC_SIO 5
#define GPIO_FUNC_PIO0 6
#define GPIO_FUNC_PIO1 7
#define GPIO_FUNC_NULL 0xf

// GPIO direction constants
#define GPIO_OUT 1
#define GPIO_IN 0

// Other constants
#define GPIO_OVERRIDE_INVERT 0x10
#define GPIO_OVERRIDE_INOVER 0x20
#define GPIO_OVERRIDE_OUTOVER 0x40
#define GPIO_OVERRIDE_OUTOVER_LSB 6
#define GPIO_OVERRIDE_INOVER_LSB 4
#define GPIO_OVERRIDE_INVERT_LSB 3

#define GPIO_SLEW_RATE_SLOW 0x80
#define GPIO_SLEW_RATE_FAST 0x00
#define GPIO_DRIVE_STRENGTH_2MA 0x00
#define GPIO_DRIVE_STRENGTH_4MA 0x40
#define GPIO_DRIVE_STRENGTH_8MA 0x80
#define GPIO_DRIVE_STRENGTH_12MA 0xc0

#define PADS_BANK0_GPIO0_PDE_BITS 0x00000004
#define PADS_BANK0_GPIO0_PUE_BITS 0x00000008

#define GPIO_OUT 1
#define GPIO_IN 0

#endif /* PICO_STDLIB_H */