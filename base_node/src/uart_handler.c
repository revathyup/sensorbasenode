#include "uart_handler.h"
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(uart_handler, LOG_LEVEL_INF);

static const struct device *const uart_dev = DEVICE_DT_GET(DT_NODELABEL(uart1));
static uint8_t rx_buf[UART_BUFFER_SIZE];
static uint8_t rx_pos;

static void uart_cb(const struct device *dev, void *user_data)
{
    uint8_t c;

    if (!uart_irq_update(dev)) {
        return;
    }

    if (uart_irq_rx_ready(dev)) {
        while (uart_fifo_read(dev, &c, 1) == 1) {
            // Log each received byte for debugging
            LOG_INF("Received byte: 0x%02x (%c)", c, isprint(c) ? c : '.');
            
            if (rx_pos < UART_BUFFER_SIZE - 1) {
                rx_buf[rx_pos++] = c;
                if (c == '\n') {
                    rx_buf[rx_pos] = '\0';
                    LOG_INF("Complete message: %s", rx_buf);
                    rx_pos = 0;
                }
            } else {
                rx_pos = 0;
            }
        }
    }
}

void uart_init(void)
{
    if (!device_is_ready(uart_dev)) {
        LOG_ERR("UART device not ready");
        return;
    }

    // Configure UART
    const struct uart_config config = {
        .baudrate = 115200,
        .parity = UART_CFG_PARITY_NONE,
        .stop_bits = UART_CFG_STOP_BITS_1,
        .data_bits = UART_CFG_DATA_BITS_8,
        .flow_ctrl = UART_CFG_FLOW_CTRL_NONE
    };

    int ret = uart_configure(uart_dev, &config);
    if (ret < 0) {
        LOG_ERR("Could not configure UART: %d", ret);
        return;
    }

    uart_irq_callback_set(uart_dev, uart_cb);
    uart_irq_rx_enable(uart_dev);
    LOG_INF("UART initialized");
}

void uart_process(void)
{
    // Print heartbeat message to confirm the handler is running
    static int count = 0;
    if (++count >= 100) {
        LOG_INF("UART handler running...");
        count = 0;
    }
    k_sleep(K_MSEC(10));
}
