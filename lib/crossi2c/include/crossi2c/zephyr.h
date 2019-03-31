#ifndef CROSSI2C_ZEPHYR_H
#define CROSSI2C_ZEPHYR_H

#include <crossi2c.h>
#include <device.h>

struct crossi2c_bus {
    struct device *i2c_dev;
};

int crossi2c_zephyr_create(struct crossi2c_bus *bus, struct device *i2c_dev);

#endif /* CROSSI2C_ZEPHYR_H */
