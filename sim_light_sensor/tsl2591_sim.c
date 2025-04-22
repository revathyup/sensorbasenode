/**
 * @file tsl2591_sim.c
 * @brief Simulated driver implementation for the TSL2591 light sensor
 */
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <unistd.h>
#include "tsl2591_sim.h"

// Static variables
static tsl2591_gain_t current_gain = TSL2591_GAIN_25X;
static tsl2591_integration_time_t current_integration_time = TSL2591_INTEGRATIONTIME_300;
static int simulation_mode = 0;  // 0=normal, 1=low light, 2=bright light, 3=random

// Integration time conversion table (in milliseconds)
static const uint16_t integration_time_ms[] = {
    100,  // TSL2591_INTEGRATIONTIME_100
    200,  // TSL2591_INTEGRATIONTIME_200
    300,  // TSL2591_INTEGRATIONTIME_300
    400,  // TSL2591_INTEGRATIONTIME_400
    500,  // TSL2591_INTEGRATIONTIME_500
    600,  // TSL2591_INTEGRATIONTIME_600
};

/**
 * @brief Initialize the simulated TSL2591 sensor
 * 
 * @return true if initialization successful, false otherwise
 */
bool tsl2591_init(void) {
    // Seed the random number generator
    srand(time(NULL));
    
    printf("TSL2591: Initialized successfully\n");
    return true;
}

/**
 * @brief Set the gain for the TSL2591 sensor
 * 
 * @param gain Gain setting
 * @return true if successful, false otherwise
 */
bool tsl2591_set_gain(tsl2591_gain_t gain) {
    current_gain = gain;
    
    // Print gain setting
    switch (gain) {
        case TSL2591_GAIN_1X:
            printf("TSL2591: Gain set to 1x\n");
            break;
        case TSL2591_GAIN_25X:
            printf("TSL2591: Gain set to 25x\n");
            break;
        case TSL2591_GAIN_428X:
            printf("TSL2591: Gain set to 428x\n");
            break;
        case TSL2591_GAIN_9876X:
            printf("TSL2591: Gain set to 9876x\n");
            break;
        default:
            printf("TSL2591: Invalid gain setting\n");
            return false;
    }
    
    return true;
}

/**
 * @brief Set the integration time for the TSL2591 sensor
 * 
 * @param integration_time Integration time setting
 * @return true if successful, false otherwise
 */
bool tsl2591_set_integration_time(tsl2591_integration_time_t integration_time) {
    if (integration_time > TSL2591_INTEGRATIONTIME_600) {
        printf("TSL2591: Invalid integration time\n");
        return false;
    }
    
    current_integration_time = integration_time;
    printf("TSL2591: Integration time set to %d ms\n", integration_time_ms[integration_time]);
    
    return true;
}

/**
 * @brief Simulate changing light levels
 * 
 * @param mode 0=normal, 1=low light, 2=bright light, 3=random
 */
void tsl2591_sim_set_mode(int mode) {
    if (mode >= 0 && mode <= 3) {
        simulation_mode = mode;
        
        switch (mode) {
            case 0:
                printf("TSL2591 Simulation: Normal light mode\n");
                break;
            case 1:
                printf("TSL2591 Simulation: Low light mode\n");
                break;
            case 2:
                printf("TSL2591 Simulation: Bright light mode\n");
                break;
            case 3:
                printf("TSL2591 Simulation: Random light mode\n");
                break;
        }
    }
}

/**
 * @brief Generate simulated light sensor readings
 * 
 * @param visible Pointer to store simulated visible reading
 * @param ir Pointer to store simulated IR reading
 */
static void generate_simulated_readings(uint16_t *visible, uint16_t *ir) {
    // Base values
    uint16_t base_visible, base_ir;
    
    // Generate readings based on simulation mode
    switch (simulation_mode) {
        case 0:  // Normal light
            base_visible = 5000 + (rand() % 2000);
            base_ir = 1000 + (rand() % 500);
            break;
        case 1:  // Low light
            base_visible = 100 + (rand() % 300);
            base_ir = 50 + (rand() % 100);
            break;
        case 2:  // Bright light
            base_visible = 30000 + (rand() % 5000);
            base_ir = 10000 + (rand() % 2000);
            break;
        case 3:  // Random (can switch between modes)
            // Randomly select from case 0, 1, or 2
            switch (rand() % 3) {
                case 0:
                    base_visible = 5000 + (rand() % 2000);
                    base_ir = 1000 + (rand() % 500);
                    break;
                case 1:
                    base_visible = 100 + (rand() % 300);
                    base_ir = 50 + (rand() % 100);
                    break;
                case 2:
                    base_visible = 30000 + (rand() % 5000);
                    base_ir = 10000 + (rand() % 2000);
                    break;
            }
            break;
        default:
            base_visible = 5000;
            base_ir = 1000;
            break;
    }
    
    // Apply gain scaling
    float gain_factor;
    switch (current_gain) {
        case TSL2591_GAIN_1X:
            gain_factor = 1.0f;
            break;
        case TSL2591_GAIN_25X:
            gain_factor = 25.0f;
            break;
        case TSL2591_GAIN_428X:
            gain_factor = 428.0f;
            break;
        case TSL2591_GAIN_9876X:
            gain_factor = 9876.0f;
            break;
        default:
            gain_factor = 1.0f;
            break;
    }
    
    // Apply integration time scaling
    float time_factor = integration_time_ms[current_integration_time] / 100.0f;
    
    // Calculate final readings
    *visible = (uint16_t)(base_visible * gain_factor * time_factor / 428.0f);
    *ir = (uint16_t)(base_ir * gain_factor * time_factor / 428.0f);
    
    // Ensure readings don't exceed maximum
    if (*visible > 65000) *visible = 65000;
    if (*ir > 65000) *ir = 65000;
}

/**
 * @brief Read the full luminosity (visible + IR) from the sensor
 * 
 * @param channel0 Pointer to store full spectrum reading
 * @param channel1 Pointer to store infrared reading
 * @return true if successful, false otherwise
 */
bool tsl2591_get_full_luminosity(uint16_t *channel0, uint16_t *channel1) {
    // Simulate a delay for the integration time
    usleep(5000);  // Just sleep for 5ms in simulation
    
    // Generate simulated visible and IR readings
    uint16_t visible, ir;
    generate_simulated_readings(&visible, &ir);
    
    // Calculate full spectrum (channel0 = visible + IR)
    *channel0 = visible + ir;
    *channel1 = ir;
    
    return true;
}

/**
 * @brief Calculate lux based on sensor readings
 * 
 * @param ch0 Full spectrum reading
 * @param ch1 Infrared reading
 * @return float Lux value
 */
float tsl2591_calculate_lux(uint16_t ch0, uint16_t ch1) {
    // Check for sensor saturation
    if ((ch0 == 0xFFFF) || (ch1 == 0xFFFF)) {
        return 0.0f;
    }
    
    // Get integration time in milliseconds
    uint16_t integration_time = integration_time_ms[current_integration_time];
    
    // Calculate gain factor
    float gain_factor;
    switch (current_gain) {
        case TSL2591_GAIN_1X:
            gain_factor = 1.0f;
            break;
        case TSL2591_GAIN_25X:
            gain_factor = 25.0f;
            break;
        case TSL2591_GAIN_428X:
            gain_factor = 428.0f;
            break;
        case TSL2591_GAIN_9876X:
            gain_factor = 9876.0f;
            break;
        default:
            gain_factor = 1.0f;
            break;
    }
    
    // Calculate lux
    float atime = integration_time / 100.0f;  // Convert to seconds
    float cpl = (atime * gain_factor) / 408.0f;
    float lux = ((float)ch0 - (float)ch1) / cpl;
    
    // Ensure lux is not negative
    if (lux < 0) {
        lux = 0;
    }
    
    return lux;
}