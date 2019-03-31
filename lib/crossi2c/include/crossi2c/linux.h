#ifndef CROSSI2C_LINUX_H
#define CROSSI2C_LINUX_H

#include <crossi2c.h>
#include <stdint.h>

struct crossi2c_bus {
    int fd;
    size_t maxtranssz;
};

int crossi2c_linux_create(struct crossi2c_bus *bus, const char *path);

#endif /* CROSSI2C_LINUX_H */
