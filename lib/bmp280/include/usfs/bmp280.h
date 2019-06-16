#ifndef USFS_BMP280_H
#define USFS_BMP280_H

#include <crossi2c.h>
#include <bmp280.h>

#define M24512_PAGESZ 128

struct usfs_bmp280 {
    struct crossi2c_bus *i2cbus;
    struct bmp280_dev bmp;
};

int usfs_bmp280_create(struct usfs_bmp280 *dev, struct crossi2c_bus *i2cbus);
int usfs_bmp280_destroy(struct usfs_bmp280 *dev);

int usfs_bmp280_init(struct usfs_bmp280 *dev);

#endif /* USFS_BMP280_H */
