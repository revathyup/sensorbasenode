/**
 * @file sensor_node.c
 * @brief Main application for sensor node
 */
#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "hardware/i2c.h"
#include "hardware/uart.h"
#include "hardware/gpio.h"
#include "hardware/adc.h"
#include "hardware/irq.h"

#include "tsl2591.h"
#include "stemma_soil.h"
#include "mcp9700.h"
#include "protocol.h"

// Pin definitions
#define I2C0_SDA_PIN 4
#define I2C0_SCL_PIN 5
#define I2C1_SDA_PIN 6
#define I2C1_SCL_PIN 7

#define UART_TX_PIN 0
#define UART_RX_PIN 1
#define INTERRUPT_PIN 2

#define TEMP_SENSOR_PIN 26  // ADC0

// UART configuration
#define UART_ID uart0
#define BAUD_RATE 115200

// Threshold default values
static uint16_t light_threshold = 1000;      // High light threshold (in lux)
static uint16_t temp_threshold = 30;         // High temperature threshold (in °C)
static uint16_t moisture_threshold = 300;    // Low moisture threshold

// Threshold types
static uint8_t light_threshold_type = THRESHOLD_TYPE_MAX;
static uint8_t temp_threshold_type = THRESHOLD_TYPE_MAX;
static uint8_t moisture_threshold_type = THRESHOLD_TYPE_MIN;

// Communication buffer
static uint8_t rx_buffer[sizeof(protocol_packet_t)];
static int rx_buffer_index = 0;
static bool packet_received = false;
static protocol_packet_t rx_packet;
static protocol_packet_t tx_packet;

// Forward declarations
static void uart_rx_handler(void);
static void process_packet(void);
static void send_packet(uint8_t cmd, const uint8_t *data, uint8_t data_len);
static bool parse_packet(void);
static bool check_thresholds_and_notify(void);
static void read_all_sensors(sensor_data_t *data);
static void configure_thresholds(const config_data_t *config);

/**
 * @brief Main application entry point
 */
int main() {
    // Initialize standard I/O over USB
    stdio_init_all();
    
    // Wait for USB connection to be established
    sleep_ms(2000);
    
    printf("Sensor Node starting...\n");
    
    // Initialize I2C0 for light sensor
    i2c_init(i2c0, 100 * 1000);
    gpio_set_function(I2C0_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(I2C0_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(I2C0_SDA_PIN);
    gpio_pull_up(I2C0_SCL_PIN);
    bi_decl(bi_2pins_with_func(I2C0_SDA_PIN, I2C0_SCL_PIN, GPIO_FUNC_I2C));
    
    // Initialize I2C1 for soil moisture sensor
    i2c_init(i2c1, 100 * 1000);
    gpio_set_function(I2C1_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(I2C1_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(I2C1_SDA_PIN);
    gpio_pull_up(I2C1_SCL_PIN);
    bi_decl(bi_2pins_with_func(I2C1_SDA_PIN, I2C1_SCL_PIN, GPIO_FUNC_I2C));
    
    // Initialize UART
    uart_init(UART_ID, BAUD_RATE);
    gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);
    uart_set_hw_flow(UART_ID, false, false);
    uart_set_format(UART_ID, 8, 1, UART_PARITY_NONE);
    uart_set_fifo_enabled(UART_ID, true);
    
    // Set up UART interrupt
    irq_set_exclusive_handler(UART0_IRQ, uart_rx_handler);
    irq_set_enabled(UART0_IRQ, true);
    uart_set_irq_enables(UART_ID, true, false);
    
    // Initialize interrupt pin
    gpio_init(INTERRUPT_PIN);
    gpio_set_dir(INTERRUPT_PIN, GPIO_OUT);
    gpio_put(INTERRUPT_PIN, 0);  // Initialize low
    
    // Initialize sensors
    printf("Initializing light sensor...\n");
    if (!tsl2591_init(i2c0)) {
        printf("Failed to initialize TSL2591 light sensor!\n");
    } else {
        printf("TSL2591 initialized successfully\n");
        tsl2591_set_gain(TSL2591_GAIN_MED);
        tsl2591_set_integration_time(TSL2591_INTEGRATIONTIME_300MS);
    }
    
    printf("Initializing soil moisture sensor...\n");
    if (!stemma_soil_init(i2c1)) {
        printf("Failed to initialize STEMMA soil moisture sensor!\n");
    } else {
        printf("STEMMA soil moisture sensor initialized successfully\n");
    }
    
    printf("Initializing temperature sensor...\n");
    if (!mcp9700_init(TEMP_SENSOR_PIN)) {
        printf("Failed to initialize MCP9700 temperature sensor!\n");
    } else {
        printf("MCP9700 temperature sensor initialized successfully\n");
    }
    
    printf("Sensor node ready!\n");
    
    while (1) {
        // Process any received packets
        if (packet_received) {
            process_packet();
            packet_received = false;
        }
        
        // Check thresholds and send interrupts if needed
        check_thresholds_and_notify();
        
        // Small delay to prevent tight looping
        sleep_ms(200);
    }
    
    return 0;
}

/**
 * @brief UART receive interrupt handler
 */
static void uart_rx_handler(void) {
    while (uart_is_readable(UART_ID)) {
        uint8_t ch = uart_getc(UART_ID);
        
        // Start of packet detection
        if (rx_buffer_index == 0 && ch != PROTOCOL_START_BYTE) {
            continue;
        }
        
        rx_buffer[rx_buffer_index++] = ch;
        
        // Check if we have received a complete header (start byte, command, length)
        if (rx_buffer_index == 3) {
            uint8_t data_length = rx_buffer[2];
            
            // Validate data length to prevent buffer overflow
            if (data_length > PROTOCOL_MAX_DATA_LENGTH) {
                // Invalid length, reset buffer
                rx_buffer_index = 0;
                continue;
            }
        }
        
        // Check if we have received a complete packet (header + data + checksum)
        if (rx_buffer_index > 3) {
            uint8_t data_length = rx_buffer[2];
            if (rx_buffer_index == data_length + 4) {  // Start + command + length + data + checksum
                // Copy to packet structure
                if (parse_packet()) {
                    packet_received = true;
                }
                
                // Reset buffer index for next packet
                rx_buffer_index = 0;
            }
        }
        
        // Buffer overflow protection
        if (rx_buffer_index >= sizeof(rx_buffer)) {
            rx_buffer_index = 0;
        }
    }
}

/**
 * @brief Parse received data into packet structure
 * 
 * @return true if packet is valid, false otherwise
 */
static bool parse_packet(void) {
    memcpy(&rx_packet, rx_buffer, sizeof(protocol_packet_t));
    
    // Verify checksum
    uint8_t calculated_checksum = 0;
    for (int i = 0; i < rx_packet.data_length + 3; i++) {
        calculated_checksum ^= rx_buffer[i];
    }
    
    if (calculated_checksum != rx_packet.checksum) {
        // Send error response
        uint8_t error_data[1] = {ERR_CHECKSUM_FAIL};
        send_packet(CMD_ERROR, error_data, 1);
        return false;
    }
    
    return true;
}

/**
 * @brief Process a received packet
 */
static void process_packet(void) {
    switch (rx_packet.command) {
        case CMD_PING: {
            // Respond with ACK
            send_packet(CMD_ACK, NULL, 0);
            break;
        }
        
        case CMD_READ_SENSOR: {
            if (rx_packet.data_length < 1) {
                uint8_t error_data[1] = {ERR_INVALID_DATA};
                send_packet(CMD_ERROR, error_data, 1);
                break;
            }
            
            uint8_t sensor_type = rx_packet.data[0];
            
            if (sensor_type == SENSOR_ALL) {
                // Read all sensors and send response
                sensor_data_t sensor_data[3];
                read_all_sensors(sensor_data);
                
                // Send response
                send_packet(CMD_SENSOR_DATA, (uint8_t *)sensor_data, sizeof(sensor_data_t) * 3);
            } else {
                // Read specific sensor
                sensor_data_t sensor_data;
                sensor_data.sensor_type = sensor_type;
                
                switch (sensor_type) {
                    case SENSOR_LIGHT: {
                        uint16_t visible = tsl2591_get_visible_light();
                        sensor_data.value = visible;
                        break;
                    }
                    
                    case SENSOR_TEMPERATURE: {
                        float temp = mcp9700_read_temperature();
                        // Convert to fixed-point (0.1°C resolution)
                        sensor_data.value = (uint16_t)(temp * 10);
                        break;
                    }
                    
                    case SENSOR_SOIL_MOISTURE: {
                        uint16_t moisture = stemma_soil_get_moisture();
                        sensor_data.value = moisture;
                        break;
                    }
                    
                    default: {
                        uint8_t error_data[1] = {ERR_INVALID_DATA};
                        send_packet(CMD_ERROR, error_data, 1);
                        return;
                    }
                }
                
                // Send response
                send_packet(CMD_SENSOR_DATA, (uint8_t *)&sensor_data, sizeof(sensor_data_t));
            }
            break;
        }
        
        case CMD_CONFIG_THRESHOLD: {
            if (rx_packet.data_length < sizeof(config_data_t)) {
                uint8_t error_data[1] = {ERR_INVALID_DATA};
                send_packet(CMD_ERROR, error_data, 1);
                break;
            }
            
            config_data_t *config = (config_data_t *)rx_packet.data;
            configure_thresholds(config);
            
            // Send ACK
            send_packet(CMD_ACK, NULL, 0);
            break;
        }
        
        case CMD_RESET: {
            // Reset thresholds to default values
            light_threshold = 1000;
            temp_threshold = 30;
            moisture_threshold = 300;
            
            light_threshold_type = THRESHOLD_TYPE_MAX;
            temp_threshold_type = THRESHOLD_TYPE_MAX;
            moisture_threshold_type = THRESHOLD_TYPE_MIN;
            
            // Send ACK
            send_packet(CMD_ACK, NULL, 0);
            break;
        }
        
        default: {
            // Unknown command
            uint8_t error_data[1] = {ERR_INVALID_COMMAND};
            send_packet(CMD_ERROR, error_data, 1);
            break;
        }
    }
}

/**
 * @brief Send a packet to the base node
 * 
 * @param cmd Command byte
 * @param data Data buffer
 * @param data_len Length of data buffer
 */
static void send_packet(uint8_t cmd, const uint8_t *data, uint8_t data_len) {
    tx_packet.start = PROTOCOL_START_BYTE;
    tx_packet.command = cmd;
    tx_packet.data_length = data_len;
    
    if (data && data_len > 0) {
        memcpy(tx_packet.data, data, data_len);
    }
    
    // Calculate checksum
    uint8_t checksum = tx_packet.start ^ tx_packet.command ^ tx_packet.data_length;
    for (int i = 0; i < data_len; i++) {
        checksum ^= tx_packet.data[i];
    }
    
    tx_packet.checksum = checksum;
    
    // Send packet over UART
    uart_write_blocking(UART_ID, (uint8_t *)&tx_packet, data_len + 4);  // Start + command + length + data + checksum
}

/**
 * @brief Read all sensors and store values in the provided array
 * 
 * @param data Array to store sensor data (must be able to hold 3 sensor_data_t structures)
 */
static void read_all_sensors(sensor_data_t *data) {
    // Read light sensor
    data[0].sensor_type = SENSOR_LIGHT;
    data[0].value = tsl2591_get_visible_light();
    
    // Read temperature sensor
    data[1].sensor_type = SENSOR_TEMPERATURE;
    float temp = mcp9700_read_temperature();
    data[1].value = (uint16_t)(temp * 10);  // Convert to fixed-point (0.1°C resolution)
    
    // Read soil moisture sensor
    data[2].sensor_type = SENSOR_SOIL_MOISTURE;
    data[2].value = stemma_soil_get_moisture();
}

/**
 * @brief Configure threshold values based on received configuration
 * 
 * @param config Pointer to configuration data
 */
static void configure_thresholds(const config_data_t *config) {
    switch (config->sensor_type) {
        case SENSOR_LIGHT:
            light_threshold = config->threshold;
            light_threshold_type = config->threshold_type;
            printf("Light threshold set to %u (type %u)\n", light_threshold, light_threshold_type);
            break;
            
        case SENSOR_TEMPERATURE:
            temp_threshold = config->threshold;
            temp_threshold_type = config->threshold_type;
            printf("Temperature threshold set to %u (type %u)\n", temp_threshold, temp_threshold_type);
            break;
            
        case SENSOR_SOIL_MOISTURE:
            moisture_threshold = config->threshold;
            moisture_threshold_type = config->threshold_type;
            printf("Moisture threshold set to %u (type %u)\n", moisture_threshold, moisture_threshold_type);
            break;
            
        default:
            printf("Invalid sensor type for threshold configuration: %u\n", config->sensor_type);
            break;
    }
}

/**
 * @brief Check if any threshold is crossed and send interrupt notification
 * 
 * @return true if interrupt was triggered, false otherwise
 */
static bool check_thresholds_and_notify(void) {
    bool interrupt_triggered = false;
    
    // Read all sensors
    uint16_t light = tsl2591_get_visible_light();
    float temp_float = mcp9700_read_temperature();
    uint16_t temp = (uint16_t)(temp_float * 10);  // Convert to fixed-point (0.1°C resolution)
    uint16_t moisture = stemma_soil_get_moisture();
    
    // Check light threshold
    bool light_trigger = false;
    if ((light_threshold_type == THRESHOLD_TYPE_MIN && light < light_threshold) ||
        (light_threshold_type == THRESHOLD_TYPE_MAX && light > light_threshold)) {
        light_trigger = true;
        printf("Light threshold crossed: %u (threshold: %u)\n", light, light_threshold);
    }
    
    // Check temperature threshold
    bool temp_trigger = false;
    if ((temp_threshold_type == THRESHOLD_TYPE_MIN && temp < temp_threshold) ||
        (temp_threshold_type == THRESHOLD_TYPE_MAX && temp > temp_threshold)) {
        temp_trigger = true;
        printf("Temperature threshold crossed: %u (threshold: %u)\n", temp, temp_threshold);
    }
    
    // Check moisture threshold
    bool moisture_trigger = false;
    if ((moisture_threshold_type == THRESHOLD_TYPE_MIN && moisture < moisture_threshold) ||
        (moisture_threshold_type == THRESHOLD_TYPE_MAX && moisture > moisture_threshold)) {
        moisture_trigger = true;
        printf("Moisture threshold crossed: %u (threshold: %u)\n", moisture, moisture_threshold);
    }
    
    // If any threshold is crossed, send interrupt notification
    if (light_trigger || temp_trigger || moisture_trigger) {
        interrupt_data_t interrupt_data;
        
        // Choose which sensor to report (prioritize in order: moisture, temperature, light)
        if (moisture_trigger) {
            interrupt_data.sensor_type = SENSOR_SOIL_MOISTURE;
            interrupt_data.value = moisture;
            interrupt_data.threshold = moisture_threshold;
            interrupt_data.threshold_type = moisture_threshold_type;
        } else if (temp_trigger) {
            interrupt_data.sensor_type = SENSOR_TEMPERATURE;
            interrupt_data.value = temp;
            interrupt_data.threshold = temp_threshold;
            interrupt_data.threshold_type = temp_threshold_type;
        } else {
            interrupt_data.sensor_type = SENSOR_LIGHT;
            interrupt_data.value = light;
            interrupt_data.threshold = light_threshold;
            interrupt_data.threshold_type = light_threshold_type;
        }
        
        // Send interrupt notification
        send_packet(CMD_INTERRUPT, (uint8_t *)&interrupt_data, sizeof(interrupt_data_t));
        
        // Toggle interrupt pin
        gpio_put(INTERRUPT_PIN, 1);
        sleep_ms(100);
        gpio_put(INTERRUPT_PIN, 0);
        
        interrupt_triggered = true;
    }
    
    return interrupt_triggered;
}
