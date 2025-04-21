/**
 * @file sensors.c
 * @brief Sensor implementation for the sensor node
 */

#include "sensors.h"
#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/adc.h"

// Global variables
static sensor_config_t g_config = {
    .temp_high_threshold = 30.0f,
    .soil_low_threshold = 30.0f,
    .light_low_threshold = 50.0f,
    .light_high_threshold = 10000.0f,
    .sampling_rate_ms = 1000,
    .int_enable_temp_high = true,
    .int_enable_soil_low = true,
    .int_enable_light_low = true,
    .int_enable_light_high = true
};

// Private function declarations
static bool tsl2591_init(void);
static bool soil_sensor_init(void);
static bool temperature_sensor_init(void);
static bool tsl2591_read(float *light);
static bool soil_sensor_read(float *moisture);
static bool temperature_sensor_read(float *temperature);
static uint8_t i2c_read_byte(uint8_t addr, uint8_t reg);
static void i2c_write_byte(uint8_t addr, uint8_t reg, uint8_t value);
static uint16_t seesaw_read_word(uint8_t base, uint8_t reg);

/**
 * @brief Initialize all sensors
 */
bool sensors_init(void) {
    // Initialize I2C
    i2c_init(I2C_PORT, I2C_FREQ);
    gpio_set_function(I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_PIN);
    gpio_pull_up(I2C_SCL_PIN);
    
    // Initialize ADC
    adc_init();
    adc_gpio_init(ADC_PIN);
    
    // Initialize individual sensors
    bool light_ok = tsl2591_init();
    bool soil_ok = soil_sensor_init();
    bool temp_ok = temperature_sensor_init();
    
    printf("Sensor initialization: Light=%s, Soil=%s, Temperature=%s\n", 
           light_ok ? "OK" : "FAIL",
           soil_ok ? "OK" : "FAIL",
           temp_ok ? "OK" : "FAIL");
    
    // Return true if at least one sensor initialized correctly
    return (light_ok || soil_ok || temp_ok);
}

/**
 * @brief Initialize TSL2591 light sensor
 */
static bool tsl2591_init(void) {
    // Check device ID
    uint8_t id = i2c_read_byte(TSL2591_I2C_ADDR, TSL2591_COMMAND_BIT | TSL2591_ID_REGISTER);
    if (id != 0x50) {
        printf("TSL2591 ID mismatch: got 0x%02x, expected 0x50\n", id);
        return false;
    }
    
    // Enable the device
    i2c_write_byte(TSL2591_I2C_ADDR, 
                  TSL2591_COMMAND_BIT | TSL2591_ENABLE_REGISTER, 
                  TSL2591_ENABLE_POWERON | TSL2591_ENABLE_AEN);
    
    // Set gain and integration time
    i2c_write_byte(TSL2591_I2C_ADDR, 
                  TSL2591_COMMAND_BIT | TSL2591_CONTROL_REGISTER, 
                  TSL2591_GAIN_MED | TSL2591_INTEGRATIONTIME_300MS);
    
    sleep_ms(100);
    return true;
}

/**
 * @brief Initialize soil moisture sensor
 */
static bool soil_sensor_init(void) {
    uint16_t hw_id = seesaw_read_word(SEESAW_STATUS_BASE, SEESAW_STATUS_HW_ID);
    if ((hw_id >> 8) != SEESAW_HW_ID_CODE) {
        printf("Soil sensor HW ID mismatch: got 0x%04x, expected 0x%02x\n", 
               hw_id, SEESAW_HW_ID_CODE);
        return false;
    }
    
    sleep_ms(100);
    return true;
}

/**
 * @brief Initialize temperature sensor
 */
static bool temperature_sensor_init(void) {
    // Nothing specific needed for analog temperature sensor
    return true;
}

/**
 * @brief Read data from all sensors
 */
bool sensors_read(sensor_data_t *data) {
    if (!data) return false;
    
    // Reset valid flags
    data->valid_light = false;
    data->valid_temperature = false;
    data->valid_soil = false;
    
    // Read light sensor
    data->valid_light = tsl2591_read(&data->light);
    
    // Read soil moisture sensor
    data->valid_soil = soil_sensor_read(&data->soil_moisture);
    
    // Read temperature sensor
    data->valid_temperature = temperature_sensor_read(&data->temperature);
    
    return (data->valid_light || data->valid_temperature || data->valid_soil);
}

/**
 * @brief Read light sensor data
 */
static bool tsl2591_read(float *light) {
    if (!light) return false;
    
    // Read channel 0 (visible + IR) and channel 1 (IR)
    uint8_t ch0_low = i2c_read_byte(TSL2591_I2C_ADDR, TSL2591_COMMAND_BIT | TSL2591_CHAN0_LOW);
    uint8_t ch0_high = i2c_read_byte(TSL2591_I2C_ADDR, TSL2591_COMMAND_BIT | TSL2591_CHAN0_HIGH);
    uint8_t ch1_low = i2c_read_byte(TSL2591_I2C_ADDR, TSL2591_COMMAND_BIT | TSL2591_CHAN1_LOW);
    uint8_t ch1_high = i2c_read_byte(TSL2591_I2C_ADDR, TSL2591_COMMAND_BIT | TSL2591_CHAN1_HIGH);
    
    uint16_t ch0 = (ch0_high << 8) | ch0_low;
    uint16_t ch1 = (ch1_high << 8) | ch1_low;
    
    // Check for overflow
    if (ch0 == 0xFFFF || ch1 == 0xFFFF) {
        printf("TSL2591 sensor overflowed\n");
        return false;
    }
    
    // Calculate lux
    float gain = 25.0f;  // Medium gain (25x)
    float integration_time = 300.0f; // 300ms integration time
    
    // Apply calibration factors
    float cpl = (integration_time * gain) / 408.0f;
    float lux1 = ((float)ch0 - (float)ch1) * (1.0f - ((float)ch1 / (float)ch0)) / cpl;
    
    // Constrain result
    if (lux1 < 0) lux1 = 0;
    
    *light = lux1;
    return true;
}

/**
 * @brief Read soil moisture sensor data
 */
static bool soil_sensor_read(float *moisture) {
    if (!moisture) return false;
    
    // Read capacitive touch reading from the soil sensor
    uint16_t cap_reading = seesaw_read_word(SEESAW_TOUCH_BASE, SEESAW_TOUCH_CHANNEL_OFFSET);
    
    // Convert to percentage - values typically range from ~200 (very dry) to ~2000 (very wet)
    // Scale to 0-100%
    if (cap_reading > 2000) cap_reading = 2000;
    if (cap_reading < 200) cap_reading = 200;
    
    *moisture = ((float)(cap_reading - 200) / 1800.0f) * 100.0f;
    return true;
}

/**
 * @brief Read temperature sensor data
 */
static bool temperature_sensor_read(float *temperature) {
    if (!temperature) return false;
    
    // Select ADC channel
    adc_select_input(TEMP_SENSOR_ADC_CHANNEL);
    
    // Read ADC value
    uint16_t adc_result = adc_read();
    
    // Convert ADC reading to voltage (in mV)
    // ADC range is 0-4095 for 0-3.3V
    float voltage_mv = (adc_result * 3300.0f) / 4096.0f;
    
    // Convert voltage to temperature
    // MCP9700 produces 10mV per °C with 500mV offset at 0°C
    *temperature = (voltage_mv - TEMP_SENSOR_OFFSET_MV) / TEMP_SENSOR_MV_PER_C;
    
    return true;
}

/**
 * @brief Set sensor configuration
 */
bool sensors_set_config(const sensor_config_t *config) {
    if (!config) return false;
    
    // Copy configuration to global config
    g_config = *config;
    
    return true;
}

/**
 * @brief Get current sensor configuration
 */
bool sensors_get_config(sensor_config_t *config) {
    if (!config) return false;
    
    // Copy global config to output
    *config = g_config;
    
    return true;
}

/**
 * @brief Check for interrupt conditions
 */
bool sensors_check_interrupts(const sensor_data_t *data, interrupt_status_t *status) {
    if (!data || !status) return false;
    
    // Reset all status flags
    status->temp_high = false;
    status->soil_low = false;
    status->light_low = false;
    status->light_high = false;
    
    // Check each sensor against threshold if valid and enabled
    if (data->valid_temperature && g_config.int_enable_temp_high) {
        status->temp_high = (data->temperature > g_config.temp_high_threshold);
    }
    
    if (data->valid_soil && g_config.int_enable_soil_low) {
        status->soil_low = (data->soil_moisture < g_config.soil_low_threshold);
    }
    
    if (data->valid_light) {
        if (g_config.int_enable_light_low) {
            status->light_low = (data->light < g_config.light_low_threshold);
        }
        if (g_config.int_enable_light_high) {
            status->light_high = (data->light > g_config.light_high_threshold);
        }
    }
    
    // Return true if any interrupt is active
    return (status->temp_high || status->soil_low || status->light_low || status->light_high);
}

/**
 * @brief Read a byte from I2C device
 */
static uint8_t i2c_read_byte(uint8_t addr, uint8_t reg) {
    uint8_t result;
    
    i2c_write_blocking(I2C_PORT, addr, &reg, 1, true);
    i2c_read_blocking(I2C_PORT, addr, &result, 1, false);
    
    return result;
}

/**
 * @brief Write a byte to I2C device
 */
static void i2c_write_byte(uint8_t addr, uint8_t reg, uint8_t value) {
    uint8_t data[2] = {reg, value};
    i2c_write_blocking(I2C_PORT, addr, data, 2, false);
}

/**
 * @brief Read a 16-bit word from seesaw I2C device
 */
static uint16_t seesaw_read_word(uint8_t base, uint8_t reg) {
    uint8_t buf[2] = {base, reg};
    uint8_t result[2];
    
    i2c_write_blocking(I2C_PORT, SOIL_SENSOR_I2C_ADDR, buf, 2, true);
    sleep_ms(5); // Small delay required for seesaw
    i2c_read_blocking(I2C_PORT, SOIL_SENSOR_I2C_ADDR, result, 2, false);
    
    return (result[0] << 8) | result[1];
}
