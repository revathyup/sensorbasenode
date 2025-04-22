/**
 * @file main.c
 * @brief Main program for the base node
 */

#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "hardware/uart.h"
#include "uart_protocol.h"
#include "../protocol.h"

// Pin definitions
#define LED_PIN 25          // Onboard LED
#define UART_TX_PIN 8
#define UART_RX_PIN 9

// Command types for the base node console
#define CMD_NONE            0
#define CMD_HELP            1
#define CMD_SHOW_SENSORS    3
#define CMD_SET_THRESHOLD   4
// Note: CMD_PING is already defined in protocol.h

// Global variables
static const uint UART_BAUD = 115200;     // 115200 baud
static uart_inst_t *uart = uart1;
static uint8_t led_state = 0;
static absolute_time_t next_led_toggle;
static absolute_time_t next_ping;

// Sensor data storage
static light_data_t last_light_data = {0};
static uint32_t last_light_update = 0;
static float last_temperature = 0.0f;
static uint32_t last_temp_update = 0;
static uint16_t last_soil_moisture = 0;
static uint32_t last_soil_update = 0;

// Threshold settings
static float light_high_threshold = 1000.0f;
static float light_low_threshold = 50.0f;
static float temp_high_threshold = 30.0f;
static float temp_low_threshold = 10.0f;
static uint16_t soil_dry_threshold = 300;
static uint16_t soil_wet_threshold = 700;

// Function prototypes
void setup_hardware(void);
void process_uart_packets(void);
void toggle_led(void);
void print_help(void);
void process_console_command(void);
int parse_console_command(char *cmd_buffer);
void send_thresholds_to_sensor(void);
void print_sensor_data(void);

int main() {
    // Initialize system
    setup_hardware();
    
    printf("\n=============================================\n");
    printf("     Smart Gardening Base Node\n");
    printf("=============================================\n");
    
    // Initialize UART protocol
    printf("Initializing UART communication...\n");
    if (!uart_protocol_init(uart, UART_BAUD, UART_TX_PIN, UART_RX_PIN)) {
        printf("Failed to initialize UART protocol!\n");
    }
    
    printf("Base node ready!\n");
    printf("Type 'help' for a list of commands.\n");
    
    // Set up timing variables
    next_led_toggle = make_timeout_time_ms(500);
    next_ping = make_timeout_time_ms(5000);  // Ping every 5 seconds
    
    // Main loop
    while (true) {
        // Process console commands
        process_console_command();
        
        // Check for incoming UART packets
        process_uart_packets();
        
        // Send periodic ping to ensure connectivity
        if (absolute_time_diff_us(get_absolute_time(), next_ping) <= 0) {
            uart_protocol_send_ping();
            next_ping = make_timeout_time_ms(5000);
        }
        
        // Toggle LED to indicate system is running
        if (absolute_time_diff_us(get_absolute_time(), next_led_toggle) <= 0) {
            toggle_led();
            next_led_toggle = make_timeout_time_ms(500);
        }
        
        // Give the processor a break
        sleep_ms(10);
    }
    
    return 0;
}

/**
 * @brief Set up hardware components
 */
void setup_hardware(void) {
    // Initialize stdio
    stdio_init_all();
    
    // Delay to allow USB to initialize
    sleep_ms(2000);
    
    // Initialize onboard LED
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
    
    // Additional hardware setup can be added here
}

/**
 * @brief Process incoming UART packets
 */
void process_uart_packets(void) {
    if (uart_protocol_data_available()) {
        protocol_packet_t packet;
        
        if (uart_protocol_receive_packet(&packet)) {
            // Process packet based on command
            switch (packet.command) {
                case CMD_SENSOR_DATA:
                    // Handle sensor data
                    if (packet.data[0] == SENSOR_LIGHT) {
                        // Parse light sensor data
                        light_data_t light_data;
                        if (uart_protocol_parse_light_data(&packet, &light_data)) {
                            // Store light data
                            memcpy(&last_light_data, &light_data, sizeof(light_data_t));
                            last_light_update = to_ms_since_boot(get_absolute_time());
                            
                            // Send ACK
                            uart_protocol_send_ack(CMD_SENSOR_DATA);
                        }
                    }
                    // Add handling for other sensor types
                    break;
                    
                case CMD_ALERT:
                    // Handle alert messages
                    if (packet.length >= 9) {
                        uint8_t alert_type = packet.data[0];
                        float value, threshold;
                        
                        // Extract value (4 bytes)
                        float *value_ptr = (float*)&packet.data[1];
                        value = *value_ptr;
                        
                        // Extract threshold (4 bytes)
                        float *threshold_ptr = (float*)&packet.data[5];
                        threshold = *threshold_ptr;
                        
                        // Process different alert types
                        switch (alert_type) {
                            case ALERT_LIGHT_HIGH:
                                printf("ALERT: Light level above threshold (%.2f lux > %.2f lux)\n", 
                                       value, threshold);
                                break;
                                
                            case ALERT_LIGHT_LOW:
                                printf("ALERT: Light level below threshold (%.2f lux < %.2f lux)\n", 
                                       value, threshold);
                                break;
                                
                            case ALERT_TEMP_HIGH:
                                printf("ALERT: Temperature above threshold (%.1f°C > %.1f°C)\n", 
                                       value, threshold);
                                break;
                                
                            case ALERT_TEMP_LOW:
                                printf("ALERT: Temperature below threshold (%.1f°C < %.1f°C)\n", 
                                       value, threshold);
                                break;
                                
                            case ALERT_SOIL_DRY:
                                printf("ALERT: Soil is too dry (%d < %d)\n", 
                                       (int)value, (int)threshold);
                                break;
                                
                            case ALERT_SOIL_WET:
                                printf("ALERT: Soil is too wet (%d > %d)\n", 
                                       (int)value, (int)threshold);
                                break;
                                
                            default:
                                printf("Unknown alert type: 0x%02X\n", alert_type);
                                break;
                        }
                        
                        // Send ACK
                        uart_protocol_send_ack(CMD_ALERT);
                    }
                    break;
                    
                case CMD_ACK:
                    // Handle acknowledgment
                    if (packet.length >= 1) {
                        printf("Received ACK for command 0x%02X\n", packet.data[0]);
                    }
                    break;
                    
                case CMD_NACK:
                    // Handle negative acknowledgment
                    if (packet.length >= 2) {
                        printf("Received NACK for command 0x%02X, error code 0x%02X\n", 
                               packet.data[0], packet.data[1]);
                    }
                    break;
                    
                case CMD_PING:
                    // Respond to ping with ACK
                    printf("Received ping, sending ACK\n");
                    uart_protocol_send_ack(CMD_PING);
                    break;
                    
                default:
                    printf("Received unknown command: 0x%02X\n", packet.command);
                    break;
            }
        }
    }
}

/**
 * @brief Toggle the onboard LED
 */
void toggle_led(void) {
    led_state = !led_state;
    gpio_put(LED_PIN, led_state);
}

/**
 * @brief Print help information
 */
void print_help(void) {
    printf("\n--- Available Commands ---\n");
    printf("help                        : Show this help message\n");
    printf("ping                        : Send ping to sensor node\n");
    printf("show                        : Show current sensor readings\n");
    printf("set light high <value>      : Set light high threshold (lux)\n");
    printf("set light low <value>       : Set light low threshold (lux)\n");
    printf("set temp high <value>       : Set temperature high threshold (°C)\n");
    printf("set temp low <value>        : Set temperature low threshold (°C)\n");
    printf("set soil dry <value>        : Set soil dry threshold (0-1000)\n");
    printf("set soil wet <value>        : Set soil wet threshold (0-1000)\n");
    printf("---------------------------\n");
}

/**
 * @brief Process commands from the console
 */
void process_console_command(void) {
    static char cmd_buffer[128];
    static int buf_pos = 0;
    
    // Check if there's data from the console
    int c = getchar_timeout_us(0);
    if (c != PICO_ERROR_TIMEOUT) {
        if (c == '\r' || c == '\n') {
            // End of command
            cmd_buffer[buf_pos] = '\0';
            
            // Process command if buffer is not empty
            if (buf_pos > 0) {
                int cmd = parse_console_command(cmd_buffer);
                
                switch (cmd) {
                    case CMD_HELP:
                        print_help();
                        break;
                        
                    case CMD_PING:
                        printf("Sending ping to sensor node...\n");
                        if (uart_protocol_send_ping()) {
                            printf("Ping sent.\n");
                        } else {
                            printf("Failed to send ping!\n");
                        }
                        break;
                        
                    case CMD_SHOW_SENSORS:
                        print_sensor_data();
                        break;
                        
                    case CMD_SET_THRESHOLD:
                        send_thresholds_to_sensor();
                        break;
                        
                    default:
                        printf("Unknown command. Type 'help' for available commands.\n");
                        break;
                }
            }
            
            // Reset buffer
            buf_pos = 0;
            printf("> ");
        } else if (c == 8 || c == 127) {
            // Backspace or delete
            if (buf_pos > 0) {
                buf_pos--;
                printf("\b \b");  // Erase character on terminal
            }
        } else if (buf_pos < sizeof(cmd_buffer) - 1) {
            // Add character to buffer
            cmd_buffer[buf_pos++] = (char)c;
            putchar(c);  // Echo character
        }
    }
}

/**
 * @brief Parse a command from the console
 * 
 * @param cmd_buffer Command string
 * @return int Command type
 */
int parse_console_command(char *cmd_buffer) {
    // Convert to lowercase for easier comparison
    for (int i = 0; cmd_buffer[i]; i++) {
        if (cmd_buffer[i] >= 'A' && cmd_buffer[i] <= 'Z') {
            cmd_buffer[i] += 32;  // Convert to lowercase
        }
    }
    
    // Compare command
    if (strcmp(cmd_buffer, "help") == 0) {
        return CMD_HELP;
    } else if (strcmp(cmd_buffer, "ping") == 0) {
        return CMD_PING;
    } else if (strcmp(cmd_buffer, "show") == 0) {
        return CMD_SHOW_SENSORS;
    } else if (strncmp(cmd_buffer, "set ", 4) == 0) {
        // Parse threshold setting command
        char param1[16], param2[16];
        float value;
        
        int result = sscanf(cmd_buffer + 4, "%s %s %f", param1, param2, &value);
        
        if (result == 3) {
            // Update appropriate threshold
            if (strcmp(param1, "light") == 0) {
                if (strcmp(param2, "high") == 0) {
                    light_high_threshold = value;
                    printf("Light high threshold set to %.2f lux\n", value);
                } else if (strcmp(param2, "low") == 0) {
                    light_low_threshold = value;
                    printf("Light low threshold set to %.2f lux\n", value);
                }
            } else if (strcmp(param1, "temp") == 0) {
                if (strcmp(param2, "high") == 0) {
                    temp_high_threshold = value;
                    printf("Temperature high threshold set to %.1f°C\n", value);
                } else if (strcmp(param2, "low") == 0) {
                    temp_low_threshold = value;
                    printf("Temperature low threshold set to %.1f°C\n", value);
                }
            } else if (strcmp(param1, "soil") == 0) {
                int int_value = (int)value;
                if (strcmp(param2, "dry") == 0) {
                    soil_dry_threshold = int_value;
                    printf("Soil dry threshold set to %d\n", int_value);
                } else if (strcmp(param2, "wet") == 0) {
                    soil_wet_threshold = int_value;
                    printf("Soil wet threshold set to %d\n", int_value);
                }
            }
            
            return CMD_SET_THRESHOLD;
        }
    }
    
    return CMD_NONE;
}

/**
 * @brief Send updated thresholds to the sensor node
 */
void send_thresholds_to_sensor(void) {
    uint8_t data[24];
    uint8_t pos = 0;
    
    // Light high threshold
    float *float_ptr = (float*)&data[pos];
    *float_ptr = light_high_threshold;
    pos += 4;
    
    // Light low threshold
    float_ptr = (float*)&data[pos];
    *float_ptr = light_low_threshold;
    pos += 4;
    
    // Temperature high threshold
    float_ptr = (float*)&data[pos];
    *float_ptr = temp_high_threshold;
    pos += 4;
    
    // Temperature low threshold
    float_ptr = (float*)&data[pos];
    *float_ptr = temp_low_threshold;
    pos += 4;
    
    // Soil dry threshold
    data[pos++] = (soil_dry_threshold >> 8) & 0xFF;
    data[pos++] = soil_dry_threshold & 0xFF;
    
    // Soil wet threshold
    data[pos++] = (soil_wet_threshold >> 8) & 0xFF;
    data[pos++] = soil_wet_threshold & 0xFF;
    
    // Send configuration command
    if (uart_protocol_send_command(CMD_SENSOR_CONFIG, data, pos)) {
        printf("Thresholds sent to sensor node.\n");
    } else {
        printf("Failed to send thresholds!\n");
    }
}

/**
 * @brief Print current sensor data
 */
void print_sensor_data(void) {
    uint32_t current_time = to_ms_since_boot(get_absolute_time());
    
    printf("\n--- Current Sensor Readings ---\n");
    
    // Light sensor data
    if (current_time - last_light_update < 10000) {  // Data less than 10 seconds old
        printf("Light Sensor:\n");
        printf("  Full: %d\n", last_light_data.full);
        printf("  IR: %d\n", last_light_data.ir);
        printf("  Visible: %d\n", last_light_data.visible);
        printf("  Lux: %.2f\n", last_light_data.lux);
    } else {
        printf("Light Sensor: No recent data\n");
    }
    
    // Temperature data
    if (current_time - last_temp_update < 10000) {
        printf("Temperature: %.1f°C\n", last_temperature);
    } else {
        printf("Temperature: No recent data\n");
    }
    
    // Soil moisture data
    if (current_time - last_soil_update < 10000) {
        printf("Soil Moisture: %d/1000\n", last_soil_moisture);
    } else {
        printf("Soil Moisture: No recent data\n");
    }
    
    printf("\n--- Current Thresholds ---\n");
    printf("Light: %.2f to %.2f lux\n", light_low_threshold, light_high_threshold);
    printf("Temperature: %.1f to %.1f°C\n", temp_low_threshold, temp_high_threshold);
    printf("Soil Moisture: %d to %d\n", soil_dry_threshold, soil_wet_threshold);
    printf("---------------------------\n");
}