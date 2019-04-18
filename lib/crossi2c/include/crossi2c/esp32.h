#ifndef CROSSI2C_ESP32_H
#define CROSSI2C_ESP32_H

#include <crossi2c.h>
#include <driver/i2c.h>

struct crossi2c_bus {
    i2c_port_t port;
};

int crossi2c_esp32_create(struct crossi2c_bus *bus, i2c_port_t port);

#endif /* CROSSI2C_ESP32_H */
