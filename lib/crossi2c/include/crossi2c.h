#ifndef CROSSI2C_H
#define CROSSI2C_H

#include <stddef.h>
#include <stdint.h>

struct crossi2c_bus;

int crossi2c_destroy(struct crossi2c_bus *bus);

int crossi2c_read(struct crossi2c_bus *bus, uint16_t addr, void *buf, size_t len);
int crossi2c_write(struct crossi2c_bus *bus, uint16_t addr, const void *buf, size_t len);
int crossi2c_write_read(struct crossi2c_bus *bus, uint16_t addr,
    const void *writebuf, size_t writelen,
    void *readbuf, size_t readlen);

static inline int crossi2c_read_byte(struct crossi2c_bus *bus, uint16_t addr, uint8_t reg, uint8_t *pvalue)
{
    return crossi2c_write_read(bus, addr,
        &reg, sizeof(reg),
        pvalue, sizeof(*pvalue));
}

static inline int crossi2c_write_byte(struct crossi2c_bus *bus, uint16_t addr, uint8_t reg, uint8_t value) {
    uint8_t tx_buf[2] = {reg, value};

    return crossi2c_write(bus, addr, tx_buf, 2);
}

#endif /* CROSSI2C_H */
