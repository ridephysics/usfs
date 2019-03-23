#include <crossi2c/linux.h>
#include <sys/ioctl.h>
#include <limits.h>
#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <string.h>
#include <linux/i2c-dev.h>

#define MIN(a,b) ((a)<(b) ? (a) : (b))

#define CROSSLOG_TAG "crossi2c"
#include <crosslog.h>

int crossi2c_linux_create(struct crossi2c_bus *bus, const char *path, size_t maxtranssz) {
    if (maxtranssz == 0) {
        maxtranssz = 64;
    }

    memset(bus, 0, sizeof(*bus));

    bus->path = path;
    bus->maxtranssz = maxtranssz;

    return 0;
}

int crossi2c_destroy(struct crossi2c_bus *bus) {
    memset(bus, 0, sizeof(*bus));
    return 0;
}

int crossi2c_open(struct crossi2c_bus *bus, struct crossi2c_dev *dev, uint8_t addr, size_t nretries) {
    int rc;
    int fd;

    memset(dev, 0, sizeof(*dev));

    fd = open(bus->path, O_RDWR, 0);
    if (fd < 0) {
        CROSSLOG_ERRNO("open");
        return -1;
    }

    rc = ioctl(fd, I2C_SLAVE, addr);
    if (rc) {
        CROSSLOG_ERRNO("ioctl");
        goto err_close;
    }

    dev->bus = bus;
    dev->fd = fd;
    dev->nretries = nretries;

    return 0;

err_close:
    close(fd);

    return -1;
}

int crossi2c_close(struct crossi2c_dev *dev) {
    close(dev->fd);
    memset(dev, 0, sizeof(*dev));

    return 0;
}

static int crossi2c_safe_read(struct crossi2c_dev *dev, void *buf, size_t len) {
    ssize_t nbytes;
    struct crossi2c_bus *bus = dev->bus;

    while (len) {
        size_t toread = MIN(len, bus->maxtranssz);
        size_t retries = dev->nretries;

        do {
            nbytes = read(dev->fd, buf, toread);
            if (nbytes < 0) {
                if (retries) {
                    retries--;
                    continue;
                }
                else {
                    return -1;
                }
            }

            if (nbytes == 0) {
                CROSSLOGE("fd got closed");
                return -1;
            }

            buf += nbytes;
            len -= nbytes;
            break;
        } while (retries);
    }

    return 0;
}

static int crossi2c_safe_write(struct crossi2c_dev *dev, const void *buf, size_t len) {
    ssize_t nbytes;
    struct crossi2c_bus *bus = dev->bus;

    while (len) {
        size_t towrite = MIN(len, bus->maxtranssz);
        size_t retries = dev->nretries;

        do {
            nbytes = write(dev->fd, buf, towrite);
            if (nbytes < 0) {
                if (retries) {
                    retries--;
                    continue;
                }
                else {
                    return -1;
                }
            }

            if (nbytes == 0) {
                CROSSLOGE("fd got closed");
                return -1;
            }

            buf += nbytes;
            len -= nbytes;
            break;
        } while (retries);
    }

    return 0;
}

int crossi2c_read(struct crossi2c_dev *dev, uint8_t reg, void *buf, size_t len) {
    int rc;

    rc = crossi2c_safe_write(dev, &reg, 1);
    if (rc) {
        CROSSLOG_ERRNO("crossi2c_safe_write");
        return -1;
    }

    rc = crossi2c_safe_read(dev, buf, len);
    if (rc) {
        CROSSLOG_ERRNO("crossi2c_safe_read");
        return -1;
    }

    return 0;
}

int crossi2c_write(struct crossi2c_dev *dev, const void *buf, size_t len) {
    return crossi2c_safe_write(dev, buf, len);
}
