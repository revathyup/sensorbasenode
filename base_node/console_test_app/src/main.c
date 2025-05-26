#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/logging/log.h>
#include <zephyr/usb/usb_device.h>

LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);

void main(void)
{
    usb_enable(NULL);
    printk("UART base node ready!\n");
    LOG_INF("UART base node ready!");

    const struct device *sensor_dev = DEVICE_DT_GET_ANY(mycompany_sensor_node);
    if (!device_is_ready(sensor_dev)) {
        LOG_ERR("Sensor node device not ready!");
        return;
    }

    while (1) {
        struct sensor_value lux;
        int ret = sensor_sample_fetch_chan(sensor_dev, SENSOR_CHAN_LIGHT);
        if (ret == 0) {
            ret = sensor_channel_get(sensor_dev, SENSOR_CHAN_LIGHT, &lux);
            if (ret == 0) {
                printk("[UART] Light: %d.%06d lux\n", lux.val1, lux.val2);
            } else {
                printk("[UART] Failed to get sensor value: %d\n", ret);
            }
        } else {
            printk("[UART] Failed to fetch sensor sample: %d\n", ret);
        }
        k_sleep(K_MSEC(1000));
    }
}