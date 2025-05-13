#include "bme680.h"
#include <stdio.h>
#include "pico/stdlib.h"

// Remove this line to avoid redefinition
// #define BME680_I2C_ADDR 0x76

static i2c_inst_t *i2c_instance;

bool bme680_init(i2c_inst_t *i2c) {
    i2c_instance = i2c;
    uint8_t chip_id;
    uint8_t reg = BME680_ID;
    
    printf("Initializing BME680...\n");
    
    // Verify device presence
    if (i2c_write_blocking(i2c_instance, BME680_I2C_ADDR, &reg, 1, true) != 1 ||
        i2c_read_blocking(i2c_instance, BME680_I2C_ADDR, &chip_id, 1, false) != 1) {
        printf("BME680 not detected at address 0x%02X\n", BME680_I2C_ADDR);
        return false;
    }
    
    printf("BME680 detected, chip ID: 0x%02X\n", chip_id);
    
    // Soft reset
    uint8_t reset_cmd[2] = {BME680_RESET, 0xB6};
    if (i2c_write_blocking(i2c_instance, BME680_I2C_ADDR, reset_cmd, sizeof(reset_cmd), false) != sizeof(reset_cmd)) {
        printf("Failed to reset BME680\n");
        return false;
    }
    sleep_ms(10);
    
    // Configure humidity oversampling (2x)
    uint8_t hum_config[2] = {BME680_CTRL_HUM, 0x02};
    if (i2c_write_blocking(i2c_instance, BME680_I2C_ADDR, hum_config, sizeof(hum_config), false) != sizeof(hum_config)) {
        printf("Failed to configure humidity oversampling\n");
        return false;
    }
    
    // Configure sensor mode and oversampling
    uint8_t meas_config[2] = {BME680_CTRL_MEAS, 0x25}; // Temp x1, Press x1, Forced mode
    if (i2c_write_blocking(i2c_instance, BME680_I2C_ADDR, meas_config, sizeof(meas_config), false) != sizeof(meas_config)) {
        printf("Failed to configure measurement mode\n");
        return false;
    }
    
    // Load calibration data after reset
    uint8_t calib_data[0xEF - 0x8A];
    uint8_t calib_reg = 0x8A;
    
    if (i2c_write_blocking(i2c_instance, BME680_I2C_ADDR, &calib_reg, 1, true) != 1 ||
        i2c_read_blocking(i2c_instance, BME680_I2C_ADDR, calib_data, sizeof(calib_data), false) != sizeof(calib_data)) {
        printf("Failed to read calibration data\n");
        return false;
    }
    
    printf("BME680 initialized successfully\n");
    return true;
}

bool bme680_sample_fetch(float *humidity) {
    uint8_t status;  // Single declaration here
    uint8_t status_reg = BME680_STATUS;
    int attempts = 0;
    
    // Trigger measurement with explicit mode setting
    uint8_t meas_cmd[2] = {BME680_CTRL_MEAS, 0x25};
    if (i2c_write_blocking(i2c_instance, BME680_I2C_ADDR, meas_cmd, sizeof(meas_cmd), false) != sizeof(meas_cmd)) {
        printf("Measurement trigger failed\n");
        return false;
    }
    
    // Extended wait with status verification
    do {
        sleep_ms(50);
        if (i2c_write_blocking(i2c_instance, BME680_I2C_ADDR, &status_reg, 1, true) != 1 ||
            i2c_read_blocking(i2c_instance, BME680_I2C_ADDR, &status, 1, false) != 1) {
            printf("Status check failed\n");
            return false;
        }
        attempts++;
    } while ((status & 0x80) == 0 && attempts < 10);
    
    // Read humidity data
    uint8_t hum_data[2];
    uint8_t hum_reg = BME680_HUM_MSB;
    
    if (i2c_write_blocking(i2c_instance, BME680_I2C_ADDR, &hum_reg, 1, true) != 1 ||
        i2c_read_blocking(i2c_instance, BME680_I2C_ADDR, hum_data, 2, false) != 2) {
        printf("Failed to read humidity data\n");
        return false;
    }
    
    // Convert to humidity percentage
    uint16_t raw_hum = (hum_data[0] << 8) | hum_data[1];
    *humidity = (float)raw_hum / 1024.0f;
    
    printf("Humidity: %.1f%%\n", *humidity);
    return true;
}