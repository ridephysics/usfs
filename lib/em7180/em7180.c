#include <em7180.h>
#include <em7180_regs.h>
#include <string.h>

#define CROSSLOG_TAG "em7180"
#include <crosslog.h>

#define EM7180_ADDRESS 0x28

static int em7180_read(struct em7180 *dev, uint8_t reg, void *buf, size_t len) {
    return crossi2c_write_read(dev->i2cbus, EM7180_ADDRESS, &reg, 1, buf, len);
}

#if 0
static int em7180_write_byte(struct em7180 *dev, uint8_t reg, uint8_t value) {
    return crossi2c_write_byte(dev->i2cbus, EM7180_ADDRESS, reg, value);
}
#endif

int em7180_create(struct em7180 *dev, struct crossi2c_bus *i2cbus) {
    memset(dev, 0, sizeof(*dev));

    dev->i2cbus = i2cbus;

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

void em7180_print_sentral_status(uint8_t status) {
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

void em7180_print_algorithm_ctrl(uint8_t ctrl) {
    CROSSLOGD("algorithm control:");

    if (ctrl & EM7180_AC_STANDBY_ENABLE)
        CROSSLOGD("\tstandby enable");
    if (ctrl & EM7180_AC_RAWDATA_ENABLE)
        CROSSLOGD("\trawdata enable");
    if (ctrl & EM7180_AC_HPR_OUTPUT)
        CROSSLOGD("\thpr output");
    if (ctrl & EM7180_AC_6AXIS_ENABLE)
        CROSSLOGD("\t6-axis enable");
    if (ctrl & EM7180_AC_RESERVED1)
        CROSSLOGD("\treserved 1");
    if (ctrl & EM7180_AC_ENU_OUTPUT)
        CROSSLOGD("\tenu output")
    if (ctrl & EM7180_AC_DIS_GYR_WHEN_STILL)
        CROSSLOGD("\tdisable gyro when still")
    if (ctrl & EM7180_AC_PARAM_TRANSFER)
        CROSSLOGD("\tparam transfer");
}

void em7180_print_algorithm_status(uint8_t status) {
    CROSSLOGD("algorithm status:");

    if (status & EM7180_AS_STANDBY)
        CROSSLOGD("\tstandby");
    if (status & EM7180_AS_SLOW)
        CROSSLOGD("\talgorithm slow");
    if (status & EM7180_AS_STILLNESS)
        CROSSLOGD("\tstillness");
    if (status & EM7180_AS_CAL_STABLE)
        CROSSLOGD("\tcal stable");
    if (status & EM7180_AS_MAG_TRANSIENT)
        CROSSLOGD("\tmag transient");
    if (status & EM7180_AS_UNRELIABLE)
        CROSSLOGD("\tunreliable sensor data")
    if (status & EM7180_AS_RESERVED1)
        CROSSLOGD("\treserved 1")
    if (status & EM7180_AS_RESERVED2)
        CROSSLOGD("\treserved 2");
}
