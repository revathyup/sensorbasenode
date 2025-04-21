/**
 * @file uart.h
 * @brief Mock Pico SDK UART hardware header for demonstration
 */

#ifndef HARDWARE_UART_H
#define HARDWARE_UART_H

#include <stdint.h>
#include <stdbool.h>

typedef struct uart_inst uart_inst_t;

// UART instances
extern uart_inst_t *uart0;
extern uart_inst_t *uart1;

// UART functions
static inline void uart_init(uart_inst_t *uart, uint baudrate) {}
static inline int uart_putc_raw(uart_inst_t *uart, uint8_t c) { return 0; }
static inline int uart_getc(uart_inst_t *uart) { return -1; }
static inline bool uart_is_readable(uart_inst_t *uart) { return false; }
static inline bool uart_is_writable(uart_inst_t *uart) { return true; }
static inline void uart_set_baudrate(uart_inst_t *uart, uint baudrate) {}
static inline void uart_set_format(uart_inst_t *uart, uint data_bits, uint stop_bits, uint parity) {}
static inline void uart_set_fifo_enabled(uart_inst_t *uart, bool enabled) {}

// Flow control defines
#define UART_PARITY_NONE 0
#define UART_PARITY_EVEN 1
#define UART_PARITY_ODD 2

#endif /* HARDWARE_UART_H */