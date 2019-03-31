#ifndef M24512_H
#define M24512_H

#include <crossi2c.h>

#define M24512_PAGESZ 128

struct m24512 {
    struct crossi2c_bus *i2cbus;
};

int m24512_create(struct m24512 *dev, struct crossi2c_bus *i2cbus);
int m24512_destroy(struct m24512 *dev);
int m24512_read(struct m24512 *dev, uint16_t addr, void *data, size_t len);

#endif /* M24512_H */
