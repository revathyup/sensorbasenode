#ifndef _BME680_H_
#define _BME680_H_

#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>

/* BME680 REGISTERS */
#define BME680_STATUS       0x73
#define BME680_RESET        0xE0
#define BME680_ID           0xD0
#define BME680_CONFIG       0x75
#define BME680_CTRL_MEAS    0x74
#define BME680_CTRL_HUM     0x72
#define BME680_CTRL_GAS_1   0x71
#define BME680_CTRL_GAS_0   0x70

// ... existing register definitions ...

struct bme680_data {
    float temperature;
    float pressure;
    float humidity;
    float gas_resistance;
};

int bme680_init(const struct device *dev);
int bme680_sample_fetch(const struct device *dev, struct bme680_data *data);

#endif /* _BME680_H_ */