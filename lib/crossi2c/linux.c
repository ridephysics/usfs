#include <crossi2c/linux.h>
#include <sys/ioctl.h>
#include <limits.h>
#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <string.h>
#include <linux/i2c-dev.h>
#include <linux/i2c.h>

#define MIN(a,b) ((a)<(b) ? (a) : (b))

#define CROSSLOG_TAG "crossi2c"
#include <crosslog.h>

int crossi2c_linux_create(struct crossi2c_bus *bus, const char *path, size_t maxtranssz) {
    int fd;

    if (maxtranssz == 0) {
        maxtranssz = 64;
    }

    memset(bus, 0, sizeof(*bus));

    fd = open(path, O_RDWR, 0);
    if (fd < 0) {
        CROSSLOG_ERRNO("open");
        return -1;
    }

    bus->fd = fd;
    bus->maxtranssz = maxtranssz;

    return 0;
}

int crossi2c_destroy(struct crossi2c_bus *bus) {
    close(bus->fd);
    memset(bus, 0, sizeof(*bus));
    return 0;
}

static int i2c_transfer(struct crossi2c_bus *bus, struct i2c_msg *msgs, size_t nmsgs) {
    int rc;

    struct i2c_rdwr_ioctl_data data = {
        .msgs = msgs,
        .nmsgs = nmsgs
    };

    rc = ioctl(bus->fd, I2C_RDWR, &data);
    if (rc < 0 || (size_t)rc != nmsgs) {
        return -1;
    }

    return 0;
}

int crossi2c_read(struct crossi2c_bus *bus, uint16_t addr, void *buf, size_t len) {
    struct i2c_msg msg;

    msg.addr = addr;
    msg.flags = I2C_M_RD | I2C_M_STOP;
    msg.len = len;
    msg.buf = buf;

    return i2c_transfer(bus, &msg, 1);
}

int crossi2c_write(struct crossi2c_bus *bus, uint16_t addr, const void *buf, size_t len) {
    struct i2c_msg msg;

    msg.addr = addr;
    msg.flags = I2C_M_STOP;
    msg.len = len;
    msg.buf = (void*)buf;

    return i2c_transfer(bus, &msg, 1);
}

int crossi2c_write_read(struct crossi2c_bus *bus, uint16_t addr,
    const void *writebuf, size_t writelen,
    void *readbuf, size_t readlen)
{
    struct i2c_msg msg[2];

    msg[0].addr = addr;
    msg[0].flags = 0;
    msg[0].len = writelen;
    msg[0].buf = (void*)writebuf;

    msg[1].addr = addr;
    msg[1].flags = I2C_M_RD | I2C_M_STOP;
    msg[1].len = readlen;
    msg[1].buf = readbuf;

    return i2c_transfer(bus, msg, 2);
}
