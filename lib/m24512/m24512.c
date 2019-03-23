#include <m24512.h>
#include <string.h>

#define CROSSLOG_TAG "m24512"
#include <crosslog.h>

#define M24512_ADDRESS 0x50

// while technically we could read up to 128 bytes,
// some i2c adapters don't support such big transactions
#define MAX_READ_SIZE 64

#define MIN(a,b) ((a)<(b) ? (a) : (b))
#define ROUNDDOWN(a, b) ((a) & ~((b)-1))

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
    int rc;
    uint8_t addrbuf[2];

    while (len) {
        uint8_t msb = (addr >> 8) & 0xff;
        uint8_t lsb = addr & 0xff;
        // never read beyond the chunk boundary
        size_t toread = MIN(len, (size_t)MAX_READ_SIZE - (lsb - ROUNDDOWN(lsb, MAX_READ_SIZE)));

        addrbuf[0] = msb;
        addrbuf[1] = lsb;

        rc = crossi2c_write_read(dev->i2cbus, M24512_ADDRESS,
            addrbuf, 2,
            data, toread);
        if (rc) {
            return -1;
        }

        data += toread;
        addr += toread;
        len -= toread;
    }

    return 0;
}
