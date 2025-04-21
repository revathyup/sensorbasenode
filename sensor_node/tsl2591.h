/**
 * @file tsl2591.h
 * @brief Driver for TSL2591 light sensor
 */
#ifndef TSL2591_H
#define TSL2591_H

#include <stdint.h>
#include <stdbool.h>
#include "hardware/i2c.h"

// I2C Address
#define TSL2591_I2C_ADDR 0x29

// TSL2591 Registers
#define TSL2591_COMMAND_BIT       0xA0 // 1010 0000: bits 7 and 5 for command normal operation
#define TSL2591_ENABLE_REGISTER   0x00
#define TSL2591_CONTROL_REGISTER  0x01
#define TSL2591_AILTL_REGISTER    0x04 // ALS interrupt low threshold low byte
#define TSL2591_AILTH_REGISTER    0x05 // ALS interrupt low threshold high byte
#define TSL2591_AIHTL_REGISTER    0x06 // ALS interrupt high threshold low byte
#define TSL2591_AIHTH_REGISTER    0x07 // ALS interrupt high threshold high byte
#define TSL2591_NPAILTL_REGISTER  0x08 // No persist ALS interrupt low threshold low byte
#define TSL2591_NPAILTH_REGISTER  0x09 // No persist ALS interrupt low threshold high byte
#define TSL2591_NPAIHTL_REGISTER  0x0A // No persist ALS interrupt high threshold low byte
#define TSL2591_NPAIHTH_REGISTER  0x0B // No persist ALS interrupt high threshold high byte
#define TSL2591_PERSIST_REGISTER  0x0C // Interrupt persistence filter
#define TSL2591_PACKAGE_ID        0x11 // Package ID
#define TSL2591_DEVICE_ID         0x12 // Device ID
#define TSL2591_STATUS_REGISTER   0x13 // Device status
#define TSL2591_C0DATAL_REGISTER  0x14 // Channel 0 ADC low data byte
#define TSL2591_C0DATAH_REGISTER  0x15 // Channel 0 ADC high data byte
#define TSL2591_C1DATAL_REGISTER  0x16 // Channel 1 ADC low data byte
#define TSL2591_C1DATAH_REGISTER  0x17 // Channel 1 ADC high data byte

// TSL2591 Enable Register bits
#define TSL2591_ENABLE_NPIEN 0x80 // No persist interrupt enable
#define TSL2591_ENABLE_SAI   0x40 // Sleep after interrupt enable
#define TSL2591_ENABLE_AIEN  0x10 // ALS interrupt enable
#define TSL2591_ENABLE_AEN   0x02 // ALS enable
#define TSL2591_ENABLE_PON   0x01 // Power on

// TSL2591 Control Register bits
#define TSL2591_SRESET       0x80 // Software reset

// TSL2591 Gain values
typedef enum {
    TSL2591_GAIN_LOW  = 0x00,    // 1x gain
    TSL2591_GAIN_MED  = 0x10,    // 25x gain
    TSL2591_GAIN_HIGH = 0x20,    // 428x gain
    TSL2591_GAIN_MAX  = 0x30,    // 9876x gain
} tsl2591_gain_t;

// TSL2591 Integration time values
typedef enum {
    TSL2591_INTEGRATIONTIME_100MS = 0x00, // 100 milliseconds
    TSL2591_INTEGRATIONTIME_200MS = 0x01, // 200 milliseconds
    TSL2591_INTEGRATIONTIME_300MS = 0x02, // 300 milliseconds
    TSL2591_INTEGRATIONTIME_400MS = 0x03, // 400 milliseconds
    TSL2591_INTEGRATIONTIME_500MS = 0x04, // 500 milliseconds
    TSL2591_INTEGRATIONTIME_600MS = 0x05, // 600 milliseconds
} tsl2591_integration_time_t;

// Function prototypes
bool tsl2591_init(i2c_inst_t *i2c_bus);
void tsl2591_set_gain(tsl2591_gain_t gain);
void tsl2591_set_integration_time(tsl2591_integration_time_t time);
bool tsl2591_get_full_luminosity(uint16_t *full, uint16_t *ir);
float tsl2591_calculate_lux(uint16_t ch0, uint16_t ch1);
uint16_t tsl2591_get_visible_light(void);
bool tsl2591_set_interrupt_threshold(uint16_t low, uint16_t high);
bool tsl2591_clear_interrupt(void);
bool tsl2591_enable_interrupt(bool enable);

#endif /* TSL2591_H */
