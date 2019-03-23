#ifndef CROSSI2C_LINUX_H
#define CROSSI2C_LINUX_H

#include <crossi2c.h>
#include <stdint.h>

struct crossi2c_bus {
    const char *path;
    size_t maxtranssz;
};

struct crossi2c_dev {
    struct crossi2c_bus *bus;
    int fd;
    size_t nretries;
};

int crossi2c_linux_create(struct crossi2c_bus *bus, const char *path, size_t maxtranssz);

#endif /* CROSSI2C_LINUX_H */
