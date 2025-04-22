/**
 * @file main.c
 * @brief Main application for simulated light sensor node
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <signal.h>
#include "tsl2591_sim.h"

// Light reading interval (ms)
#define LIGHT_READING_INTERVAL_MS 2000

// Light threshold for alerts (in lux)
#define LIGHT_LOW_THRESHOLD 20.0f
#define LIGHT_HIGH_THRESHOLD 1000.0f

// Flag to indicate if the program should continue running
volatile sig_atomic_t keep_running = 1;

/**
 * @brief Signal handler for clean shutdown
 */
void handle_signal(int signal) {
    keep_running = 0;
}

/**
 * @brief Process user commands from input
 * 
 * @param cmd The command string to process
 */
void process_command(const char *cmd) {
    if (strcmp(cmd, "help") == 0) {
        printf("\nCommand List:\n");
        printf("  help    - Show this help message\n");
        printf("  normal  - Set normal light conditions\n");
        printf("  low     - Set low light conditions\n");
        printf("  bright  - Set bright light conditions\n");
        printf("  random  - Set random light conditions\n");
        printf("  quit    - Exit the program\n\n");
    } else if (strcmp(cmd, "normal") == 0) {
        tsl2591_sim_set_mode(0);
    } else if (strcmp(cmd, "low") == 0) {
        tsl2591_sim_set_mode(1);
    } else if (strcmp(cmd, "bright") == 0) {
        tsl2591_sim_set_mode(2);
    } else if (strcmp(cmd, "random") == 0) {
        tsl2591_sim_set_mode(3);
    } else if (strcmp(cmd, "quit") == 0 || strcmp(cmd, "exit") == 0) {
        keep_running = 0;
    } else {
        printf("Unknown command: %s. Type 'help' for a list of commands.\n", cmd);
    }
}

/**
 * @brief Main application entry point
 */
int main() {
    // Set up signal handling for clean shutdown
    signal(SIGINT, handle_signal);
    
    printf("\n=============================================\n");
    printf("     Smart Gardening Light Sensor Node\n");
    printf("=============================================\n\n");
    
    printf("Initializing TSL2591 light sensor...\n");
    if (!tsl2591_init()) {
        printf("Failed to initialize TSL2591 light sensor!\n");
        return -1;
    }
    
    // Configure sensor settings
    tsl2591_set_gain(TSL2591_GAIN_25X);
    tsl2591_set_integration_time(TSL2591_INTEGRATIONTIME_300);
    
    printf("\nLight sensor node ready!\n");
    printf("Type 'help' for a list of commands.\n\n");
    
    // Set up non-blocking input
    fd_set readfds;
    struct timeval timeout;
    char cmd_buffer[64] = {0};
    int cmd_pos = 0;
    
    // Main loop
    time_t last_reading_time = 0;
    
    while (keep_running) {
        time_t current_time = time(NULL);
        
        // Take a sensor reading every LIGHT_READING_INTERVAL_MS
        if (current_time - last_reading_time >= LIGHT_READING_INTERVAL_MS / 1000) {
            // Read light sensor values
            uint16_t full, ir;
            if (tsl2591_get_full_luminosity(&full, &ir)) {
                // Calculate lux value
                float lux = tsl2591_calculate_lux(full, ir);
                
                // Get visible light
                uint16_t visible = full - ir;
                
                // Display readings
                printf("\nLight readings: Full: %u, IR: %u, Visible: %u, Lux: %.2f\n",
                       full, ir, visible, lux);
                
                // Check thresholds
                if (lux < LIGHT_LOW_THRESHOLD) {
                    printf("ALERT: Light level below threshold (%.2f lux < %.2f lux)\n",
                           lux, LIGHT_LOW_THRESHOLD);
                } else if (lux > LIGHT_HIGH_THRESHOLD) {
                    printf("ALERT: Light level above threshold (%.2f lux > %.2f lux)\n", 
                           lux, LIGHT_HIGH_THRESHOLD);
                }
            } else {
                printf("Failed to read from light sensor\n");
            }
            
            last_reading_time = current_time;
        }
        
        // Check for user input with a short timeout
        FD_ZERO(&readfds);
        FD_SET(STDIN_FILENO, &readfds);
        
        timeout.tv_sec = 0;
        timeout.tv_usec = 100000;  // 100ms
        
        int ready = select(STDIN_FILENO + 1, &readfds, NULL, NULL, &timeout);
        
        if (ready > 0 && FD_ISSET(STDIN_FILENO, &readfds)) {
            char c;
            if (read(STDIN_FILENO, &c, 1) > 0) {
                // Process character
                if (c == '\n') {
                    // Process command
                    cmd_buffer[cmd_pos] = '\0';
                    printf("\n"); // Add a newline after command for cleaner output
                    process_command(cmd_buffer);
                    // Reset buffer
                    memset(cmd_buffer, 0, sizeof(cmd_buffer));
                    cmd_pos = 0;
                } else if (c == 127 || c == 8) {  // Backspace
                    if (cmd_pos > 0) {
                        cmd_pos--;
                        cmd_buffer[cmd_pos] = '\0';
                        printf("\b \b");  // Erase character on screen
                    }
                } else if (cmd_pos < sizeof(cmd_buffer) - 1) {
                    cmd_buffer[cmd_pos++] = c;
                    printf("%c", c);      // Echo character
                }
            }
        }
        
        // Small delay to avoid hogging CPU
        usleep(10000);  // 10ms
    }
    
    printf("\nSensor node shutting down...\n");
    
    return 0;
}