#include <crossi2c/linux.h>
#include <em7180.h>
#include <m24512.h>
#include <fcntl.h>

#define CROSSLOG_TAG "eeprom_dump"
#include <crosslog.h>

static struct crossi2c_bus i2cbus;
static struct em7180 em7180;
static struct m24512 m24512;

int main(int argc, char **argv) {
    int rc;
    int ret = -1;
    int fd;
    uint8_t eepromdata[128];
    ssize_t nbytes;

    if (argc != 3) {
        fprintf(stderr, "Usage: %s I2CDEV OUTFILE\n", argv[0]);
        return -1;
    }

    rc = crossi2c_linux_create(&i2cbus, argv[1], 100);
    if (rc) {
        CROSSLOGE("can't create i2cbus");
        return -1;
    }

    rc = em7180_create(&em7180, &i2cbus);
    if (rc) {
        CROSSLOGE("can't create em7180 dev");
        goto out_i2cbus_destroy;
    }

    rc = m24512_create(&m24512, &i2cbus);
    if (rc) {
        CROSSLOGE("can't create m24512 dev");
        goto out_em7180_destroy;
    }

    rc = em7180_passthrough_enter(&em7180);
    if (rc) {
        CROSSLOGE("can't enter passthrough mode")
        goto out_m24512_destroy;
    }

    fd = open(argv[2], O_WRONLY|O_CREAT|O_TRUNC, 0644);
    if (fd < 0) {
        CROSSLOGE("can't open %s", argv[2]);
        goto out_leave_passthrough;
    }

    for (size_t i = 0; i < 512; i++) {
        rc = m24512_read(&m24512, i * 128, eepromdata, sizeof(eepromdata));
        if (rc) {
            CROSSLOGE("can't read from 0x%04zx", i * 128);
            goto out_leave_passthrough;
        }

        nbytes = write(fd, eepromdata, sizeof(eepromdata));
        if (nbytes != sizeof(eepromdata)) {
            CROSSLOG_ERRNO("write");
            goto out_leave_passthrough;
        }
    }

    close(fd);

    ret = 0;

out_leave_passthrough:
    rc = em7180_passthrough_exit(&em7180);
    if (rc) {
        CROSSLOGW("can't exit passthrough mode");
    }

out_m24512_destroy:
    rc = m24512_destroy(&m24512);
    if (rc) {
        CROSSLOGW("can't destroy m24512 dev");
    }

out_em7180_destroy:
    rc = em7180_destroy(&em7180);
    if (rc) {
        CROSSLOGW("can't destroy em7180 dev");
    }

out_i2cbus_destroy:
    rc = crossi2c_destroy(&i2cbus);
    if (rc) {
        CROSSLOGW("can't destroy i2cbus");
    }

    return ret;
}
