/**
 * @file gpio.h
 * @brief Mock Pico SDK GPIO hardware header for demonstration
 */

#ifndef HARDWARE_GPIO_H
#define HARDWARE_GPIO_H

#include <stdint.h>
#include <stdbool.h>

// GPIO functions
static inline void gpio_set_irq_enabled(uint gpio, uint32_t events, bool enabled) {}
static inline void gpio_set_irq_enabled_with_callback(uint gpio, uint32_t events, bool enabled, void (*callback)(uint gpio, uint32_t events)) {}
static inline void gpio_acknowledge_irq(uint gpio, uint32_t events) {}

// GPIO IRQ events
#define GPIO_IRQ_LEVEL_LOW 0x1
#define GPIO_IRQ_LEVEL_HIGH 0x2
#define GPIO_IRQ_EDGE_FALL 0x4
#define GPIO_IRQ_EDGE_RISE 0x8

#endif /* HARDWARE_GPIO_H */