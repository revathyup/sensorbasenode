/**
 * @file mcp9700.c
 * @brief Driver implementation for MCP9700 temperature sensor
 */
#include "mcp9700.h"
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <string.h>

LOG_MODULE_REGISTER(mcp9700, CONFIG_LOG_DEFAULT_LEVEL);

// MCP9700 has a 10mV/°C output with a 500mV offset at 0°C
#define MCP9700_TEMP_COEF_MV 10.0f
#define MCP9700_OFFSET_MV    500.0f

// Static variables
static const struct device *adc_device = NULL;
static uint8_t adc_channel = 0;
static struct adc_channel_cfg channel_cfg;
static struct adc_sequence sequence;
static uint16_t sample_buffer;

/**
 * @brief Initialize the MCP9700 temperature sensor
 * 
 * @param adc_dev ADC device
 * @param channel ADC channel
 * @return true if initialization successful, false otherwise
 */
bool mcp9700_init(const struct device *adc_dev, uint8_t channel) {
    adc_device = adc_dev;
    adc_channel = channel;
    
    if (!device_is_ready(adc_device)) {
        LOG_ERR("MCP9700: ADC device not ready");
        return false;
    }
    
    // Configure ADC channel
    memset(&channel_cfg, 0, sizeof(channel_cfg));
    
    // Use 12-bit ADC for Raspberry Pi Pico
    channel_cfg.gain = ADC_GAIN_1;
    channel_cfg.reference = ADC_REF_INTERNAL;
    channel_cfg.acquisition_time = ADC_ACQ_TIME_DEFAULT;
    channel_cfg.channel_id = adc_channel;
    channel_cfg.differential = false;
    
    if (adc_channel_setup(adc_device, &channel_cfg) < 0) {
        LOG_ERR("MCP9700: Failed to setup ADC channel");
        return false;
    }
    
    // Configure ADC sequence
    memset(&sequence, 0, sizeof(sequence));
    sequence.channels = BIT(adc_channel);
    sequence.buffer = &sample_buffer;
    sequence.buffer_size = sizeof(sample_buffer);
    sequence.resolution = 12;
    sequence.oversampling = 0;
    sequence.calibrate = false;
    
    LOG_INF("MCP9700: Initialized successfully");
    return true;
}

/**
 * @brief Read temperature from the MCP9700 sensor
 * 
 * @return float Temperature in Celsius
 */
float mcp9700_read_temperature(void) {
    int ret;
    float voltage_mv, temperature;
    
    ret = adc_read(adc_device, &sequence);
    if (ret < 0) {
        LOG_ERR("MCP9700: Failed to read ADC: %d", ret);
        return -273.15f; // Absolute zero error value
    }
    
    // Convert ADC reading to voltage in millivolts
    // For 12-bit ADC, max value is 4095 for Vref (3.3V typical for Pico)
    // Formula: voltage_mv = (sample_value * 3300) / 4095
    voltage_mv = ((float)sample_buffer * 3300.0f) / 4095.0f;
    
    // Convert voltage to temperature
    // Formula: temperature = (voltage_mv - 500) / 10.0
    temperature = (voltage_mv - MCP9700_OFFSET_MV) / MCP9700_TEMP_COEF_MV;
    
    LOG_DBG("MCP9700: ADC=%u, Voltage=%0.2fmV, Temperature=%0.2f°C", 
            sample_buffer, voltage_mv, temperature);
    
    return temperature;
}