#include <m24512.h>
#include <string.h>

#define CROSSLOG_TAG "m24512"
#include <crosslog.h>

#define M24512_ADDRESS 0x50

int m24512_create(struct m24512 *dev, struct crossi2c_bus *i2cbus) {
    memset(dev, 0, sizeof(*dev));

    dev->i2cbus = i2cbus;
    return 0;
}

int m24512_destroy(struct m24512 *dev) {
    memset(dev, 0, sizeof(*dev));
    return 0;
}

int m24512_read(struct m24512 *dev, uint16_t addr, void *data, size_t len) {
    uint8_t addrbuf[2];

    addrbuf[0] = (addr >> 8) & 0xff;
    addrbuf[1] = (addr) & 0xff;

    return crossi2c_write_read(dev->i2cbus, M24512_ADDRESS,
        addrbuf, 2,
        data, len);
}
