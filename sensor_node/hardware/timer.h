/**
 * @file timer.h
 * @brief Mock Pico SDK timer hardware header for demonstration
 */

#ifndef HARDWARE_TIMER_H
#define HARDWARE_TIMER_H

#include <stdint.h>
#include <stdbool.h>
#include "../pico/stdlib.h"

// Timer functions
static inline bool time_reached(absolute_time_t t) { return false; }
static inline absolute_time_t make_timeout_time_ms(uint32_t ms) { return (absolute_time_t){0}; }
static inline absolute_time_t make_timeout_time_us(uint64_t us) { return (absolute_time_t){0}; }
static inline int64_t absolute_time_diff_us(absolute_time_t from, absolute_time_t to) { return 0; }
static inline uint32_t absolute_time_diff_ms(absolute_time_t from, absolute_time_t to) { return 0; }

#endif /* HARDWARE_TIMER_H */