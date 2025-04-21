/**
 * @file i2c.h
 * @brief Mock Pico SDK I2C hardware header for demonstration
 */

#ifndef HARDWARE_I2C_H
#define HARDWARE_I2C_H

#include <stdint.h>
#include <stdbool.h>

typedef struct i2c_inst i2c_inst_t;

// I2C instances
extern i2c_inst_t *i2c0;
extern i2c_inst_t *i2c1;

// I2C functions
static inline void i2c_init(i2c_inst_t *i2c, uint baudrate) {}
static inline int i2c_write_blocking(i2c_inst_t *i2c, uint8_t addr, const uint8_t *src, size_t len, bool nostop) { return 0; }
static inline int i2c_read_blocking(i2c_inst_t *i2c, uint8_t addr, uint8_t *dst, size_t len, bool nostop) { return 0; }
static inline void i2c_set_baudrate(i2c_inst_t *i2c, uint baudrate) {}
static inline void i2c_set_slave_mode(i2c_inst_t *i2c, bool slave, uint8_t addr) {}

#endif /* HARDWARE_I2C_H */