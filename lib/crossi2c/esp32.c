#include <crossi2c/esp32.h>
#include <string.h>

#define CROSSLOG_TAG "crossi2c"
#include <crosslog.h>

int crossi2c_esp32_create(struct crossi2c_bus *bus, i2c_port_t port) {
    memset(bus, 0, sizeof(*bus));
    bus->port = port;
    return 0;
}

int crossi2c_destroy(struct crossi2c_bus *bus) {
    memset(bus, 0, sizeof(*bus));
    return 0;
}

int crossi2c_read(struct crossi2c_bus *bus, uint16_t addr, void *buf, size_t len) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, addr << 1 | I2C_MASTER_READ, true);
    if (len > 1)
        i2c_master_read(cmd, buf, len - 1, I2C_MASTER_ACK);
    i2c_master_read_byte(cmd, buf + len - 1, I2C_MASTER_NACK);
    i2c_master_stop(cmd);

    esp_err_t ret = i2c_master_cmd_begin(bus->port, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);

    if (ret) {
        CROSSLOGE("i2c_master_cmd_begin: %s(%d)", esp_err_to_name(ret), ret);
        return -1;
    }
    return 0;
}

int crossi2c_write(struct crossi2c_bus *bus, uint16_t addr, const void *buf, size_t len) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write(cmd, (void*)buf, len, true);
    i2c_master_stop(cmd);

    esp_err_t ret = i2c_master_cmd_begin(bus->port, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);

    if (ret) {
        CROSSLOGE("i2c_master_cmd_begin: %s(%d)", esp_err_to_name(ret), ret);
        return -1;
    }
    return 0;
}

int crossi2c_write_read(struct crossi2c_bus *bus, uint16_t addr,
    const void *writebuf, size_t writelen,
    void *readbuf, size_t readlen)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write(cmd, (void*)writebuf, writelen, true);

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_READ, true);
    if (readlen > 1)
        i2c_master_read(cmd, readbuf, readlen - 1, I2C_MASTER_ACK);
    i2c_master_read_byte(cmd, readbuf + readlen - 1, I2C_MASTER_NACK);

    i2c_master_stop(cmd);

    esp_err_t ret = i2c_master_cmd_begin(bus->port, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);

    if (ret) {
        CROSSLOGE("i2c_master_cmd_begin: %s(%d)", esp_err_to_name(ret), ret);
        return -1;
    }

    return 0;
}
