/**
 * @file main.c
 * @brief Main program for the sensor node
 */

#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "hardware/i2c.h"
#include "hardware/adc.h"
#include "hardware/uart.h"
#include "tsl2591.h"
#include "mcp9700.h"
#include "uart_protocol.h"
#include "../protocol.h"
#include "bme680.h"
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/usb/usb_device.h>

// Pin definitions
#define I2C_SDA_PIN 4
#define I2C_SCL_PIN 5
#define TEMP_SENSOR_PIN 26  // ADC0
#define LED_PIN 25          // Onboard LED
#define UART_TX_PIN 0
#define UART_RX_PIN 1
#define BME680_SDA_PIN 2    // Groove 2 SDA
#define BME680_SCL_PIN 3    // Groove 2 SCL

// Sensor thresholds
#define LIGHT_HIGH_THRESHOLD 1000.0f    // lux
#define LIGHT_LOW_THRESHOLD 50.0f       // lux
#define TEMP_HIGH_THRESHOLD 30.0f       // °C
#define TEMP_LOW_THRESHOLD 10.0f        // °C
#define SOIL_DRY_THRESHOLD 300          // Scaled value 0-1000
#define SOIL_WET_THRESHOLD 700          // Scaled value 0-1000

// Global variables
static const uint I2C_FREQ = 100 * 1000;  // 100 KHz
static const uint UART_BAUD = 115200;     // 115200 baud
static i2c_inst_t *i2c = i2c0;
static i2c_inst_t *bme680_i2c = i2c1;     // Second I2C for BME680
static uart_inst_t *uart = uart1;
static uint8_t led_state = 0;
static absolute_time_t next_led_toggle;
static absolute_time_t next_sensor_read;
static absolute_time_t next_console_update;

// Function prototypes
void setup_hardware(void);
void read_and_process_sensors(void);
void process_uart_commands(void);
void toggle_led(void);
void read_bme680_sensor(void);

// ... (all your includes and definitions remain unchanged)

int main() {
    // Initialize system
    setup_hardware();
    
    // Initialize BME680 I2C
    i2c_init(bme680_i2c, I2C_FREQ);
    gpio_set_function(BME680_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(BME680_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(BME680_SDA_PIN);
    gpio_pull_up(BME680_SCL_PIN);
    
    // Initialize BME680
    if (!bme680_init(bme680_i2c)) {
        // printf("Failed to initialize BME680 sensor!\\n");
    } else {
        // printf("BME680 sensor initialized successfully\\n");
    }
    
    // printf("\\n=============================================\\n");
    // printf("     Smart Gardening Sensor Node\\n");
    // printf("=============================================\\n");
    
    // printf("Initializing sensors...\\n");
    
    // printf("Initializing TSL2591 light sensor...\\n");
    if (!tsl2591_init(i2c)) {
        // printf("Failed to initialize TSL2591 light sensor!\\n");
    } else {
        tsl2591_set_gain(TSL2591_GAIN_1X);
        tsl2591_set_integration_time(TSL2591_INTEGRATIONTIME_100);
    }
    
    // printf("Initializing MCP9700 temperature sensor...\\n");
    if (!mcp9700_init(TEMP_SENSOR_PIN)) {
        // printf("Failed to initialize MCP9700 temperature sensor!\\n");
    }
    
    // printf("Initializing STEMMA soil moisture sensor...\\n");
    // if (!stemma_soil_init(i2c)) { ... }
    
    // printf("Initializing UART communication...\\n");
    if (!uart_protocol_init(uart, UART_BAUD, UART_TX_PIN, UART_RX_PIN)) {
        // printf("Failed to initialize UART protocol!\\n");
    }
    
    // printf("Sensor node ready!\\n");
    
    // Set up timing variables
    next_led_toggle = make_timeout_time_ms(500);
    next_sensor_read = make_timeout_time_ms(1000);
    next_console_update = make_timeout_time_ms(2000);
    
    // Main loop
    while (true) {
        read_and_process_sensors();
        sleep_ms(1000);
        // Commented out: timing variables and other logic for this direct sensor-to-base test
        // if (absolute_time_diff_us(get_absolute_time(), next_sensor_read) <= 0) {
        //     read_and_process_sensors();
        //     next_sensor_read = make_timeout_time_ms(1000);
        // }
        // process_uart_commands();
        // if (absolute_time_diff_us(get_absolute_time(), next_led_toggle) <= 0) {
        //     toggle_led();
        //     next_led_toggle = make_timeout_time_ms(500);
        // }
        // sleep_ms(10);
    }
    
    return 0;
}

void setup_hardware(void) {
    // Initialize stdio
    stdio_init_all();

    // Delay to allow USB to initialize
    sleep_ms(2000);

    // Initialize onboard LED
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);

    // Initialize I2C
    i2c_init(i2c, I2C_FREQ);
    gpio_set_function(I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_PIN);
    gpio_pull_up(I2C_SCL_PIN);

    // Additional hardware setup can be added here
}

void read_and_process_sensors(void) {
    // Read light sensor
    uint16_t full, ir, visible;
    float lux;
    static float prev_lux = 0;
    
    if (tsl2591_get_full_luminosity(&full, &ir)) {
        visible = full - ir;
        lux = tsl2591_calculate_lux(full, ir);
        light_data_t light_data = {
            .sensor_type = SENSOR_LIGHT,
            .full = full,
            .ir = ir,
            .visible = visible,
            .lux = lux
        };
        printf("[DEBUG] Sending light data: %f\n", lux);
        uart_protocol_send_light_data(&light_data);
        prev_lux = lux;
    }
    
    // Read temperature sensor
    float temperature = mcp9700_read_temperature();
    static float prev_temp = 0;
    temperature_data_t temp_data = {
        .sensor_type = SENSOR_TEMPERATURE,
        .temperature = temperature
    };
    printf("[DEBUG] Sending temperature data: %f\n", temperature);
    // If you have a function to send temperature, call it here
    // uart_protocol_send_temperature_data(&temp_data);
    prev_temp = temperature;

    // Read BME680 sensor (humidity)
    read_bme680_sensor();
}

void process_uart_commands(void) {
    if (uart_protocol_data_available()) {
        protocol_packet_t packet;
        
        if (uart_protocol_receive_packet(&packet)) {
            // Process packet based on command
            switch (packet.command) {
                case CMD_SENSOR_CONFIG:
                    // printf("Received sensor configuration command\\n");
                    break;
                    
                case CMD_PING:
                    // printf("Received ping, sending ACK\\n");
                    // Send ACK packet
                    break;
                    
                default:
                    // printf("Received unknown command: 0x%02X\\n", packet.command);
                    break;
            }
        }
    }
}

void read_bme680_sensor(void) {
    float humidity;
    static float prev_hum = 0;
    
    if (bme680_sample_fetch(&humidity)) {
        // printf("Raw humidity reading: %.1f%%\\n", humidity);
        // Prepare BME680 data structure
        bme680_data_t bme680_data = {
            .sensor_type = SENSOR_BME680,
            .humidity = humidity
        };
        
        // Send BME680 data over UART
        uart_protocol_send_bme680_data(&bme680_data);
        
        // Print BME680 data to console
        // if (absolute_time_diff_us(get_absolute_time(), next_console_update) <= 0) {
        //     printf("BME680 Humidity: %.1f%%\\n", humidity);
        // }
    } else {
        // printf("Failed to read from BME680 sensor\\n");
    }
    
    // In main(), after initializing I2C:
    // printf("Scanning I2C bus for BME680...\\n");
    for(uint8_t addr = 0x76; addr <= 0x77; addr++) {
        uint8_t reg = BME680_ID;
        if(i2c_write_blocking(bme680_i2c, addr, &reg, 1, true) == 1 &&
           i2c_read_blocking(bme680_i2c, addr, &reg, 1, false) == 1) {
            // printf("BME680 detected at 0x%02X\\n", addr);
        }
    }
}
