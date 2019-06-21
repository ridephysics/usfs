#ifndef USFS_BMP280_H
#define USFS_BMP280_H

#include <crossi2c.h>
#include <bmp280.h>

int usfs_bmp280_create(struct bmp280_dev *dev, struct crossi2c_bus *i2cbus);

#endif /* USFS_BMP280_H */
