#ifndef UART_HANDLER_H
#define UART_HANDLER_H

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>

#define UART_BUFFER_SIZE 256

void uart_init(void);
void uart_process(void);

#endif /* UART_HANDLER_H */
