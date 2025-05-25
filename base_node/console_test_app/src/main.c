#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/logging/log.h>
#include <zephyr/usb/usb_device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/sys/printk.h>
#include <string.h>

LOG_MODULE_REGISTER(main, CONFIG_LOG_DEFAULT_LEVEL);

#define UART_DEVICE_NODE DT_NODELABEL(uart0)
#define UART_BUF_SIZE 64
#define USB_INIT_DELAY_MS 5000

// Define sensor data structure
struct sensor_data {
    double temperature;  // Changed to double to match printf format
    double humidity;     // Changed to double to match printf format
    double light;        // Changed to double to match printf format
};

// Function to parse sensor data from UART
static int parse_sensor_data(const uint8_t *data, size_t len, struct sensor_data *sensor_data)
{
    char *str = (char *)data;
    char *token;
    int values[3] = {0};
    int i = 0;

    // Split string by commas
    token = strtok(str, ",");
    while (token != NULL && i < 3) {
        values[i++] = atoi(token);
        token = strtok(NULL, ",");
    }

    if (i == 3) {
        // Convert raw values to actual measurements
        sensor_data->temperature = values[0] / 100.0;
        sensor_data->humidity = values[1] / 100.0;
        sensor_data->light = values[2] / 100.0;
        return 0;
    }

    return -1;
}

// Function to check if USB device is ready
static bool is_usb_ready(void)
{
    const struct device *usb_dev = DEVICE_DT_GET(DT_NODELABEL(cdc_acm_uart0));
    return device_is_ready(usb_dev);
}

int main(void)
{
    const struct device *uart_dev = DEVICE_DT_GET(UART_DEVICE_NODE);
    uint8_t rx_buf[UART_BUF_SIZE + 1];
    struct sensor_data sensor_data;
    int ret;

    // Print startup message using printk (works before USB is ready)
    printk("\n\n=== Sensor Base Node Starting ===\n");
    printk("Initializing USB...\n");

    // Enable USB CDC ACM with retry
    for (int i = 0; i < 3; i++) {
        ret = usb_enable(NULL);
        if (ret == 0) {
            break;
        }
        printk("USB enable attempt %d failed, retrying...\n", i + 1);
        k_sleep(K_MSEC(1000));
    }

    if (ret != 0) {
        printk("Failed to enable USB after 3 attempts\n");
        return -1;
    }

    printk("USB enabled, waiting for device to be ready...\n");
    
    // Wait for USB to be ready
    for (int i = 0; i < 10; i++) {
        if (is_usb_ready()) {
            printk("USB device is ready!\n");
            break;
        }
        printk("Waiting for USB device... (%d/10)\n", i + 1);
        k_sleep(K_MSEC(500));
    }

    // Additional delay to ensure USB enumeration
    k_sleep(K_MSEC(USB_INIT_DELAY_MS));

    // Now switch to LOG_* for USB CDC output
    LOG_INF("=== Sensor Base Node Ready ===");
    LOG_INF("USB CDC ACM initialized successfully");
    LOG_INF("Waiting for UART data...");

    if (!device_is_ready(uart_dev)) {
        LOG_ERR("UART device not ready");
        return -1;
    }

    LOG_INF("UART device is ready");

    while (1) {
        int bytes = uart_fifo_read(uart_dev, rx_buf, UART_BUF_SIZE);
        if (bytes > 0) {
            rx_buf[bytes] = '\0'; // Null-terminate for printing
            
            // Parse sensor data
            if (parse_sensor_data(rx_buf, bytes, &sensor_data) == 0) {
                LOG_INF("Temperature: %.2f°C, Humidity: %.2f%%, Light: %.2f lux",
                    sensor_data.temperature,
                    sensor_data.humidity,
                    sensor_data.light);
            } else {
                // If parsing fails, just print raw data
                LOG_INF("Raw UART data: %s", rx_buf);
            }
        }
        k_sleep(K_MSEC(100));
    }

    return 0;
}