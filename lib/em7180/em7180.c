#include <em7180.h>
#include <em7180_regs.h>
#include <string.h>

#define CROSSLOG_TAG "em7180"
#include <crosslog.h>

#define EM7180_ADDRESS 0x28

int em7180_open(struct em7180 *dev) {
    int rc;

    dev->nopen++;

    if (dev->nopen > 1)
        return 0;

    CROSSLOGD("open");

    rc = crossi2c_open(dev->i2cbus, dev->i2cdev, EM7180_ADDRESS, 100);
    if (rc) {
        CROSSLOGE("can't open em7180 dev");
        return -1;
    }

    return 0;
}

int em7180_close(struct em7180 *dev) {
    int rc;

    if (dev->nopen == 0) {
        CROSSLOGE("em7180 is already closed");
        return -1;
    }

    dev->nopen--;
    if (dev->nopen > 0)
        return 0;

    CROSSLOGD("close");

    rc = crossi2c_close(dev->i2cdev);
    if (rc) {
        CROSSLOGE("can't close em7180 dev");
        return -1;
    }

    return 0;
}

static int em7180_read(struct em7180 *dev, uint8_t reg, void *buf, size_t len) {
    if (dev->nopen <= 0) {
        CROSSLOGE("em7180 was never opened");
        return -1;
    }

    return crossi2c_read(dev->i2cdev, reg, buf, len);
}

#if 0
static int em7180_write(struct em7180 *dev, const void *buf, size_t len) {
    if (dev->nopen <= 0) {
        CROSSLOGE("em7180 was never opened");
        return -1;
    }

    return crossi2c_write(dev->i2cdev, buf, len);
}
#endif

int em7180_create(struct em7180 *dev, struct crossi2c_bus *i2cbus, struct crossi2c_dev *i2cdev_mem) {
    memset(dev, 0, sizeof(*dev));

    dev->i2cbus = i2cbus;
    dev->i2cdev = i2cdev_mem;
    dev->nopen = 0;

    return 0;
}

int em7180_destroy(struct em7180 *dev) {
    memset(dev, 0, sizeof(*dev));
    return 0;
}

int em7180_get_romversion(struct em7180 *dev, uint16_t *pversion) {
    int rc;
    uint8_t version_raw[2];

    rc = em7180_read(dev, EM7180_REG_ROM_VERSION, &version_raw, sizeof(version_raw));
    if (rc) return -1;

    *pversion = (version_raw[0] & 0xff) << 8 | (version_raw[1] & 0xff);

    return 0;
}

int em7180_get_ramversion(struct em7180 *dev, uint16_t *pversion) {
    int rc;
    uint8_t version_raw[2];

    rc = em7180_read(dev, EM7180_REG_RAM_VERSION, &version_raw, sizeof(version_raw));
    if (rc) return -1;

    *pversion = (version_raw[0] & 0xff) << 8 | (version_raw[1] & 0xff);

    return 0;
}

int em7180_get_product_id(struct em7180 *dev, uint8_t *pversion) {
    return em7180_read(dev, EM7180_REG_PRODUCT_ID, pversion, sizeof(*pversion));
}

int em7180_get_revision_id(struct em7180 *dev, uint8_t *pversion) {
    return em7180_read(dev, EM7180_REG_REVISION_ID, pversion, sizeof(*pversion));
}

int em7180_get_feature_flags(struct em7180 *dev, uint8_t *pflags) {
    return em7180_read(dev, EM7180_REG_FEATURE_FLAGS, pflags, sizeof(*pflags));
}

int em7180_get_sentral_status(struct em7180 *dev, uint8_t *pstatus) {
    return em7180_read(dev, EM7180_REG_SENTRAL_STATUS, pstatus, sizeof(*pstatus));
}

void em7180_print_feature_flags(uint8_t flags) {
    CROSSLOGD("features:");

    if (flags & EM7180_FF_BAROMETER)
        CROSSLOGD("\tbarometer");
    if (flags & EM7180_FF_HUMIDITY_SENSOR)
        CROSSLOGD("\thumidity sensor");
    if (flags & EM7180_FF_TEMP_SENSOR)
        CROSSLOGD("\ttemperature sensor");
    if (flags & EM7180_FF_CUSTOM_SENSOR_1)
        CROSSLOGD("\tcustom sensor 1");
    if (flags & EM7180_FF_CUSTOM_SENSOR_2)
        CROSSLOGD("\tcustom sensor 2");
    if (flags & EM7180_FF_CUSTOM_SENSOR_3)
        CROSSLOGD("\tcustom sensor 3");
    if (flags & EM7180_FF_RESERVED1)
        CROSSLOGD("\treserved 1")
    if (flags & EM7180_FF_RESERVED2)
        CROSSLOGD("\treserved 2");
}

void em7180_print_sentral_status(uint8_t status){
    CROSSLOGD("sentral status:");

    if (status & EM7180_SS_EEPROM_DETECTED)
        CROSSLOGD("\teeprom detected");
    if (status & EM7180_SS_EEPROM_CFG_UPLOADED)
        CROSSLOGD("\teeprom config uploaded");
    if (status & EM7180_SS_EEPROM_CRC_ERR)
        CROSSLOGD("\teeprom crc error");
    if (status & EM7180_SS_INITIALIZED)
        CROSSLOGD("\tinitialized");
    if (status & EM7180_SS_NO_EEPROM)
        CROSSLOGD("\tno eeprom");
    if (status & EM7180_SS_RESERVED1)
        CROSSLOGD("\treserved 1")
    if (status & EM7180_SS_RESERVED2)
        CROSSLOGD("\treserved 2")
    if (status & EM7180_SS_RESERVED3)
        CROSSLOGD("\treserved 3");
}
