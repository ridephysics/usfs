#ifndef CROSSI2C_H
#define CROSSI2C_H

#include <sys/types.h>
#include <stdint.h>

struct crossi2c_bus;
struct crossi2c_dev;

int crossi2c_destroy(struct crossi2c_bus *bus);

int crossi2c_open(struct crossi2c_bus *bus, struct crossi2c_dev *dev, uint8_t addr, size_t nretries);
int crossi2c_close(struct crossi2c_dev *dev);
int crossi2c_read(struct crossi2c_dev *dev, uint8_t reg, void *buf, size_t len);
int crossi2c_write(struct crossi2c_dev *dev, const void *buf, size_t len);

#endif /* CROSSI2C_H */
