#include <crossi2c/zephyr.h>
#include <i2c.h>
#include <string.h>

#define CROSSLOG_TAG "crossi2c"
#include <crosslog.h>

int crossi2c_zephyr_create(struct crossi2c_bus *bus, struct device *i2c_dev) {
    memset(bus, 0, sizeof(*bus));
    bus->i2c_dev = i2c_dev;
    return 0;
}

int crossi2c_destroy(struct crossi2c_bus *bus) {
    memset(bus, 0, sizeof(*bus));
    return 0;
}

int crossi2c_read(struct crossi2c_bus *bus, uint16_t addr, void *buf, size_t len) {
    return i2c_read(bus->i2c_dev, buf, len, addr);
}

int crossi2c_write(struct crossi2c_bus *bus, uint16_t addr, const void *buf, size_t len) {
    return i2c_write(bus->i2c_dev, buf, len, addr);
}

int crossi2c_write_read(struct crossi2c_bus *bus, uint16_t addr,
    const void *writebuf, size_t writelen,
    void *readbuf, size_t readlen)
{
    return i2c_write_read(bus->i2c_dev, addr, writebuf, writelen, readbuf, readlen);
}
