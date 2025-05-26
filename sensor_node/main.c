#include "pico/stdlib.h"
#include "hardware/uart.h"
#include "uart_protocol.h"
#include "tsl2591.h"

#define UART_ID uart0
#define BAUD_RATE 115200
#define UART_TX_PIN 0
#define UART_RX_PIN 1
#define I2C_ID i2c0
#define I2C_SDA_PIN 4
#define I2C_SCL_PIN 5

int main() {
    stdio_init_all();
    uart_protocol_init(UART_ID, BAUD_RATE, UART_TX_PIN, UART_RX_PIN);

    // Initialize I2C for TSL2591
    i2c_init(I2C_ID, 100 * 1000);
    gpio_set_function(I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_PIN);
    gpio_pull_up(I2C_SCL_PIN);

    // Initialize TSL2591 sensor
    bool sensor_ok = tsl2591_init(I2C_ID);
    if (!sensor_ok) {
        printf("TSL2591 init failed!\n");
    }

    while (true) {
        light_data_t light_data;
        bool read_ok = false;
        if (sensor_ok) {
            uint16_t full = 0, ir = 0;
            read_ok = tsl2591_get_full_luminosity(&full, &ir);
            light_data.sensor_type = 1; // 1 = TSL2591
            light_data.full = full;
            light_data.ir = ir;
            light_data.visible = full > ir ? (full - ir) : 0;
            light_data.lux = tsl2591_calculate_lux(full, ir);
            printf("[DEBUG] Sending sensor data: full=%u ir=%u visible=%u lux=%f\n", light_data.full, light_data.ir, light_data.visible, light_data.lux);
        }
        if (!read_ok) {
            // Use dummy values if sensor read fails
            light_data.sensor_type = 1;
            light_data.full = 1234;
            light_data.ir = 567;
            light_data.visible = 890;
            light_data.lux = 42.0f;
            printf("[DEBUG] TSL2591 read failed, sending dummy data: full=%u ir=%u visible=%u lux=%f\n", light_data.full, light_data.ir, light_data.visible, light_data.lux);
        }
        uart_protocol_send_light_data(&light_data);
        sleep_ms(1000);
    }
}